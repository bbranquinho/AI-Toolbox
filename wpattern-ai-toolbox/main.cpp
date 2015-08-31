#include <iostream>
#include <fstream>

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>

#include <AIToolbox/MDP/Model.hpp>
#include <AIToolbox/POMDP/Algorithms/PBVI.hpp>
#include <AIToolbox/POMDP/Model.hpp>

using namespace std;

using namespace boost;
using namespace boost::program_options;

using namespace AIToolbox;

enum ALGORITHM_TYPE {
  PBVI
};

static map<string, ALGORITHM_TYPE> mapAlgorithmTypes {
  { "PBVI", ALGORITHM_TYPE::PBVI }
};

// Methods declaration.
POMDP::Model<MDP::Model> loadModel(string pomdpFile);
bool processCommandLine(int argc, char** argv, string& algorithm, string& pomdpFile, string& outputFile);
void solvePomdp(string algorithm, string pomdpFile, string outputFile);

/**
 * MAIN
 */
int main(int argc, char **argv) {
  string algorithm;
  string pomdpFile;
  string outputFile;
  
  processCommandLine(argc, argv, algorithm, pomdpFile, outputFile);
  solvePomdp(algorithm, pomdpFile, outputFile);
  
  return 0;
}


POMDP::Model<MDP::Model> loadModel(string pomdpFile) {
    size_t S = 2, A = 3, O = 2;

    POMDP::Model<MDP::Model> model(O, S, A);

    string line;
    ifstream textFile(pomdpFile);
    
    if (textFile.is_open()) {
      while (getline(textFile,line)) {
	typedef tokenizer<boost::char_separator<char>> tokenizer;
	char_separator<char> sep(":");
	tokenizer tokens(line, sep);
	tokenizer::iterator tok_iter = tokens.begin();
	
	if (tok_iter != tokens.end()) {
	  string headerStr = *tok_iter;
	  
	  for (; tok_iter != tokens.end(); ++tok_iter) {
	    cout << *tok_iter << endl;
	  }
	}
	cout << "\n";
      }
      textFile.close();
    } else {
      cout << "Unable to open file '" << pomdpFile << "'." << endl;
      exit(1);
    }
    
    return model;
}


bool processCommandLine(int argc, char** argv, string& algorithm, string& pomdpFile, string& outputFile) {
  // Declare the supported options.
  options_description desc("Allowed options");
  positional_options_description p;
  p.add("pomdp-file", -1);
  
  // Configure the options.
  desc.add_options()
    ("algorithm", value<string>(&algorithm)->default_value("PBVI"), "POMDP algorithm to solve the problem. The algorithm is 'PBVI' by default.")
    ("help", "Display this help message and exit.")
    ("output-file", value<string>(&outputFile)->default_value("out.policy"), "Use policyFileName as the name of policy output file. The file name is 'out.policy' by default.")
    ("pomdp-file", value<string>(&pomdpFile)->required(), "POMDP file with a pomdp model.");

  variables_map vm;
  store(command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
  
  if (vm.count("help")) {
      cout << desc << "\n";
      return 1;
  }
  
  notify(vm);
    
  return true;
}

void solvePomdp(string algorithm, string pomdpFile, string outputFile) {
  loadModel(pomdpFile);
  
  algorithm = to_upper_copy<string>(algorithm);
  
  switch (mapAlgorithmTypes[algorithm]) {
    case PBVI:
      
      break;
  };
}
