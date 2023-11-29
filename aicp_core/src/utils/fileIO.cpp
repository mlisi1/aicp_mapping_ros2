#include "aicp_core/aicp_utils/fileIO.h"

/* Note: it skips first lines (assuming header). */
string readLineFromFile(string& filename, int line_number)
{
  string lines, line;
  int index = 0;

  ifstream trasfFile (filename);
  if (trasfFile.is_open())
  {
    while ( getline (trasfFile, lines) )
    {
      if (index == line_number)
      {
        getline (trasfFile, line);
      }
      index ++;
    }
    trasfFile.close();
  }
  else cout << "[File IO] Unable to open file " << filename << "." << endl;

  return line;
}

/* Get a transformation Eigen::Matrix4f
given a string containing index, translation, quaternions: id x y z w x y z */
Eigen::Matrix4f parseTransformationQuaternions(string transform)
{
  Eigen::Matrix4f parsedTrans = Eigen::Matrix4f::Identity(4,4);

  float transValues[8] = {0};
  stringstream transStringStream(transform);
  for( int i = 0; i < 8; i++) {
    if(!(transStringStream >> transValues[i])) {
      cerr << "[File IO] Unable to parse ground truth poses." << endl;
      return parsedTrans;
    }
  }

  Eigen::Vector3f transl;
  transl << transValues[1], transValues[2], transValues[3];
  Eigen::Quaternionf quat;
  quat.w() = transValues[4]; quat.x() = transValues[5];
  quat.y() = transValues[6]; quat.z() = transValues[7];

//  cout << "[File IO] quat: " << quat.w() << " "
//                             << quat.x() << " "
//                             << quat.y() << " "
//                             << quat.z() << " "
//                             << endl;
//  cout << "[File IO] transl: " << transl << endl;

  parsedTrans.block<3,3>(0,0) = quat.toRotationMatrix();
  parsedTrans.block<3,1>(0,3) = transl.transpose();

  return parsedTrans;
}

/* Get a transformation matrix (Eigen)
given a string containing info about a translation on the plane x-y and a rotation
about the vertical axis z, i.e. [x,y,theta] (meters,meters,degrees). */
Eigen::Matrix4f parseTransformationDeg(string& transform)
{
  Eigen::Matrix4f parsedTrans = Eigen::Matrix4f::Identity(4,4);

  transform.erase(std::remove(transform.begin(), transform.end(), '['),
            transform.end());
  transform.erase(std::remove(transform.begin(), transform.end(), ']'),
            transform.end());
  std::replace( transform.begin(), transform.end(), ',', ' ');
  std::replace( transform.begin(), transform.end(), ';', ' ');

  float transValues[3] = {0};
  stringstream transStringStream(transform);
  for( int i = 0; i < 3; i++) {
    if(!(transStringStream >> transValues[i])) {
      cerr << "[File IO] No transformation to be parsed." << endl;
      return parsedTrans;
    }
  }

  for( int i = 0; i < 3; i++) {
    if (i == 2)
    {
      parsedTrans(i-2,i-2) = cos ( transValues[i] * M_PI / 180.0 );
      parsedTrans(i-2,i-1) = - sin ( transValues[i] * M_PI / 180.0 );
      parsedTrans(i-1,i-2) = sin ( transValues[i] * M_PI / 180.0 );
      parsedTrans(i-1,i-1) = cos ( transValues[i] * M_PI / 180.0 );
    }
    else
    {
      parsedTrans(i,3) = transValues[i];
    }
  }

  //cout << "Parsed initial transformation:" << endl << parsedTrans << endl;

  return parsedTrans;
}

/* Given a transformation matrix write line to file
 * in the format: idA idB x y z w x y z. The function adds lines
to the same file until the program ends.*/
void write3DTransformToFile(Eigen::Matrix4f &transform, string out_file, int id_cloud_A, int id_cloud_B)
{
  ofstream file (out_file);

  Eigen::Quaternionf quat(transform.block<3,3>(0,0));

  if (file.is_open())
  {
    //file << "idA idB x y z w x y z\n";
    file << id_cloud_A << " " << id_cloud_B << " "
         << transform(0,3) << " " << transform(1,3) << " " << transform(2,3) << " "
         << quat.w() << " " << quat.x() << " " << quat.y() << " " << quat.z() << endl;

    file.close();
  }
  else cout << "[File IO] Unable to open output file.\n";
  cout << "[File IO] Written file: " << out_file << endl;
}

/* Given a transformation matrix write line to file
 * in the format: id x y theta (meters,meters,radians), where id is a number
from the concatenation of the indexes of the registered clouds. The function adds lines
to the same file until the program ends.*/
void writeTransformToFile(Eigen::MatrixXf &transformations, string out_file, int num_clouds)
{
  std::vector<string> v;
  for (int i = 0; i < num_clouds-1; i++)
  {
    for (int j = 1+i; j < num_clouds; j++)
    {
      string str;
      str.append(to_string(i));
      str.append(to_string(j));
      v.push_back(str);
    }
  }

  ofstream file (out_file);

  if (file.is_open())
  {
    //file << "# x y theta\n";
    for (int i = 0; i < v.size(); i++)
    {
      file << v[i] << " " << transformations(0,i) << " " << transformations(1,i)
      << " " << transformations(2,i) << endl;
    }
    file.close();
  }
  else cout << "[File IO] Unable to open file.\n";
  cout << "[File IO] Written file: " << out_file << endl;
}

void writeLineToFile(Eigen::MatrixXf &values, string out_file, int line_number)
{
  // given values (row vector) store it as a line in a file with line number as first value
  ofstream file (out_file, std::ios_base::app | std::ios_base::out);

  if (file.is_open())
  {
    //file << "# 0 1 ... \n";
    file << line_number << " ";
    for (int i = 0; i < values.cols(); i++)
    {
      file << values(0,i) << " ";
    }
    file << endl;
    file.close();
  }
  else cout << "[File IO] Unable to open file.\n";
//  cout << "[File IO] Written file: " << out_file << endl;
}

void replaceRatioConfigFile(string in_file, string out_file, float ratio)
{
  std::ifstream in;
  in.open(in_file, std::fstream::in);
  std::ofstream out;
  out.open(out_file, std::ofstream::out);
  if (!in)
  {
    cerr << "[File IO] Could not open config file for params update." << "\n";
  }
  if (!out)
  {
    cerr << "[File IO] Could not open config file for params update." << "\n";
  }

  string word_to_replace = "ratio: ";
  std::stringstream word_to_replace_with_tmp;
  word_to_replace_with_tmp << "ratio: ";
  word_to_replace_with_tmp << ratio;
  string word_to_replace_with = word_to_replace_with_tmp.str();

  string line;
  size_t len = word_to_replace.length() + 4;
  while (!in.eof())
  {
    getline(in, line);
    size_t pos = line.find(word_to_replace);
    if (pos != string::npos)
    {
      line.replace(pos, len, word_to_replace_with);
    }
    out << line << '\n';
  }
  in.close();
  out.close();
}

// Update the input string.
void autoExpandEnvironmentVariables( std::string & text ) {
    static std::regex env( "\\$\\{([^}]+)\\}" );
    std::smatch match;
    while ( std::regex_search( text, match, env ) ) {
        const char * s = getenv( match[1].str().c_str() );
        const std::string var( s == NULL ? "" : s );
        text.replace( match[0].first, match[0].second, var );
    }
}

// Leave input alone and return new string.
std::string expandEnvironmentVariables( const std::string & input ) {
    std::string text = input;
    autoExpandEnvironmentVariables( text );
    return text;
}
