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
using namespace boost::algorithm;

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
vector<string> parseToken(string line);
double **buildUniformMatrix(int numLines, int numCols);
double **buildIndetityMatrix(int numLines, int numCols);
int indexOf(vector<string> values, string value);
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
    ifstream textFile(pomdpFile);
    char_separator<char> sep(":");
    string token, line;
    vector<string> states, actions, observations;
    double discount;
    Table3D *transitionsProbabilities, *observationsProbabilities, *rewards;

    if (textFile.is_open()) {
      while (getline(textFile,line)) {
	trim(line);
	
	if ((line.size() > 0) && (line[0] != '#')) {
	  typedef tokenizer<boost::char_separator<char>> tokenizer;
	  tokenizer tokens(line, sep);
	  tokenizer::iterator tok_iter = tokens.begin();
	  
	  if (tok_iter != tokens.end()) {
	    token = to_lower_copy<string>(*tok_iter);
	    trim(token);
	    
	    if (token.compare("discount") == 0) {
	      token = *++tok_iter;
	      
	      if (tok_iter != tokens.end()) {
		discount = stod(token);
	      }
	    }
	    
	    if (token.compare("values") == 0) {
	      // Do nothing.
	    }
	    
	    if (token.compare("states") == 0) {
	      states = parseToken(*++tok_iter);
	    }
	    
	    if (token.compare("actions") == 0) {
	      actions = parseToken(*++tok_iter);
	    }
	    
	    if (token.compare("observations") == 0) {
	      observations = parseToken(*++tok_iter);
	    }
	    
	    if (token.compare("t") == 0) {
	      int numStates = states.size(), numActions = actions.size();
	      string actionValue = *++tok_iter;
	      
	      transitionsProbabilities = new Table3D(extents[numStates][numActions][numStates]);
	      
	      int actionIndex = indexOf(actions, actionValue);
	      
	      if ((actionIndex < 0) || (actionIndex >= numActions)) {
		throw std::invalid_argument("Action [" + actionValue + "] not mapped.");
	      }
	      
	      if (!getline(textFile,line)) {
		throw std::invalid_argument("Transitions not available.");
	      }
	      
	      trim(line);
	      line = to_lower_copy<string>(line);
	      double **matrix;
	      
	      if (line.compare("uniform") == 0) {
		matrix = buildUniformMatrix(numStates, numStates);
	      } else if (line.compare("identity") == 0) {
		matrix = buildIndetityMatrix(numStates, numStates);
	      } else {
		
	      }
	      
	      for (int i = 0; i < numStates; i++) {
		for (int j = 0; j < numStates; j++) {
		  //transitionsProbabilities[i][actionIndex][j] = 0.0;// matrix[i][j];
		}
	      }
	      
	      delete matrix;
	    }
	    
	    if (token.compare("o") == 0) {
	      observationsProbabilities = new Table3D(extents[states.size()][actions.size()][observations.size()]);
	      
	    }
	    
	    if (token.compare("r") == 0) {
	      rewards = new Table3D(extents[states.size()][actions.size()][states.size()]);
	      
	    }
	  }
	}
      }
      textFile.close();
    } else {
      cout << "Unable to open file '" << pomdpFile << "'." << endl;
      exit(1);
    }
    
    POMDP::Model<MDP::Model> model(observations.size(), states.size(), actions.size());
    
    model.setDiscount(discount);
    model.setTransitionFunction(*transitionsProbabilities);
    model.setObservationFunction(*observationsProbabilities);
    model.setRewardFunction(*rewards);
    
    delete transitionsProbabilities;
    delete observationsProbabilities;
    delete rewards;
    
    return model;
}

double **buildUniformMatrix(int numLines, int numCols) {
  double **matrix;
  
  matrix = new double*[numLines];
  
  for (size_t i = 0; i < numLines; i++) {
    matrix[i] = new double[numCols];
    for (size_t j = 0; j < numCols; j++) {
      matrix[i][j] = 1.0 / numCols;
    }
  }
  
  return matrix;
}

double **buildIndetityMatrix(int numLines, int numCols) {
  double **matrix;
  
  matrix = new double*[numLines];
  
  for (size_t i = 0; i < numLines; i++) {
    matrix[i] = new double[numCols];
    for (size_t j = 0; j < numCols; j++) {
      matrix[i][j] = (i == j) ? 1.0 : 0.0;
    }
  }
  
  return matrix;
}

int indexOf(vector<string> values, string value) {
  value = to_lower_copy<string>(value);
  
  for (size_t i = 0; i < values.size(); i++) {
    if (values[i].compare(value) == 0) {
      return i;
    }
  }
  
  return -1;
}

vector<string> parseToken(string token) {
  vector<string> states;
  string value;
  
  char_separator<char> sep(" ");
  typedef tokenizer<boost::char_separator<char>> tokenizer;
  tokenizer tokens(token, sep);
  tokenizer::iterator tok_iter = tokens.begin();
  
  for (tok_iter = tokens.begin(); tok_iter != tokens.end(); tok_iter++) {
    value = to_lower_copy<string>(*tok_iter);
    trim<string>(value);
    states.push_back(value);
  }
  
  return states;
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
