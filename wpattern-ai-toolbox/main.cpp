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
double **buildMatrix(int numStates, string firstData, ifstream& textFile);
double **buildUniformMatrix(int numLines, int numCols);
double **buildIndetityMatrix(int numLines, int numCols);
int indexOf(vector<string> values, string value);
void solvePomdp(string algorithm, string pomdpFile, string outputFile);
vector<int> retrieveIndexes(string token, vector<string> names);
double retrieveValue(string token);
void printModel(POMDP::Model<MDP::Model> model);

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

void solvePomdp(string algorithm, string pomdpFile, string outputFile) {
  auto model = loadModel(pomdpFile);
  
  printModel(model);
  
  algorithm = to_upper_copy<string>(algorithm);
  
  switch (mapAlgorithmTypes[algorithm]) {
    case PBVI:
      unsigned horizon = 5;
      POMDP::PBVI solver(1000, horizon, 0.01);
      
      auto solution = solver(model);
      auto & vf = get<1>(solution);
      auto vlist = vf[horizon];
      
      // Save the solution.
      ofstream outputStream;
      outputStream.open(outputFile);
      
      for ( size_t i = 0; i < vlist.size(); ++i ) {
	auto action = get<POMDP::ACTION>(vlist[i]);
	auto observations = get<POMDP::OBS>(vlist[i]);
	auto values = get<POMDP::VALUES>(vlist[i]);
	
	outputStream << action << endl;
	
	for ( long j = 0; j < values.cols(); j++ ) {
	  for ( long k = 0; k < values.rows(); k++ ) {
	    outputStream << values.coeff(k, j) << " ";
	  }
	  outputStream << endl << endl;
	}
	outputStream << endl;
      }
      outputStream.close();
      break;
  };
}

POMDP::Model<MDP::Model> loadModel(string pomdpFile) {
    char_separator<char> sep(":");
    string token, line;
    vector<string> states, actions, observations;
    double discount;

    // Read the pomdp file to get the "discount", "states", "actions" and "observations".
    ifstream textFile(pomdpFile);
    if (textFile.is_open()) {
      while (getline(textFile,line)) {
	trim(line);
	
	if ((line.size() > 0) && (line[0] != '#')) {
	  typedef tokenizer<char_separator<char>> tokenizer;
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
	    
	    if (token.compare("states") == 0) {
	      states = parseToken(*++tok_iter);
	    }
	    
	    if (token.compare("actions") == 0) {
	      actions = parseToken(*++tok_iter);
	    }
	    
	    if (token.compare("observations") == 0) {
	      observations = parseToken(*++tok_iter);
	    }
	  }
	}
      }
      
      textFile.close();
    } else {
      cout << "Unable to open file '" << pomdpFile << "'." << endl;
      exit(1);
    }
    
    // Read the pomdp file to get the "discount", "states", "actions" and "observations".
    int numStates = states.size(), numActions = actions.size(), numObs = observations.size();
    
    Table3D transitionProbabilities(extents[numStates][numActions][numStates]);
    Table3D observationProbabilities(extents[numStates][numActions][numObs]);
    Table3D rewards(extents[numStates][numActions][numStates]);
    
    ifstream textFile2(pomdpFile);
    if (textFile2.is_open()) {
      while (getline(textFile2,line)) {
	trim(line);
	
	if ((line.size() > 0) && (line[0] != '#')) {
	  typedef tokenizer<char_separator<char>> tokenizer;
	  tokenizer tokens(line, sep);
	  tokenizer::iterator tok_iter = tokens.begin();
	  
	  if (tok_iter != tokens.end()) {
	    token = to_lower_copy<string>(*tok_iter);
	    trim(token);
	    
	    if (token.compare("t") == 0) {
	      int numStates = states.size(), numActions = actions.size();
	      
	      int actionIndex = indexOf(actions, *++tok_iter);
	      
	      if ((actionIndex < 0) || (actionIndex >= numActions)) {
		throw std::invalid_argument("Action [" + *tok_iter + "] not mapped.");
	      }
	      
	      if (!getline(textFile2, line)) {
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
		matrix = buildMatrix(numStates, line, textFile2);
	      }
	      
	      for (int i = 0; i < numStates; i++) {
		for (int j = 0; j < numStates; j++) {
		  transitionProbabilities[i][actionIndex][j] = matrix[i][j];
		}
	      }
	      
	      delete matrix;
	    }
	    
	    if (token.compare("o") == 0) {
	      int actionIndex = indexOf(actions, *++tok_iter);
	      
	      if ((actionIndex < 0) || (actionIndex >= numActions)) {
		throw std::invalid_argument("Action [" + *tok_iter + "] not mapped.");
	      }
	      
	      if (!getline(textFile2,line)) {
		throw std::invalid_argument("Transitions not available.");
	      }
	      
	      trim(line);
	      line = to_lower_copy<string>(line);
	      double **matrix;
	      
	      if (line.compare("uniform") == 0) {
		matrix = buildUniformMatrix(numStates, numObs);
	      } else if (line.compare("identity") == 0) {
		matrix = buildIndetityMatrix(numStates, numObs);
	      } else {
		matrix = buildMatrix(numStates, line, textFile2);
	      }
	      
	      for (int i = 0; i < numStates; i++) {
		for (int j = 0; j < numObs; j++) {
		  observationProbabilities[i][actionIndex][j] = matrix[i][j];
		}
	      }
	      
	      delete matrix;
	    }
	    
	    if (token.compare("r") == 0) {
	      // R: action : start-state : end-state : observation %f
	      vector<int> actionIndexes = retrieveIndexes(*++tok_iter, actions);
	      vector<int> startStateIndexes = retrieveIndexes(*++tok_iter, states);
	      vector<int> endStateIndexes = retrieveIndexes(*++tok_iter, states);
	      double rewardValue = retrieveValue(*++tok_iter);
	      
	      for (size_t a = 0; a < actionIndexes.size(); a++) { // action
		for (size_t ss = 0; ss < startStateIndexes.size(); ss++) { // start state
		  for (size_t es = 0; es < endStateIndexes.size(); es++) { // end state
		    rewards[startStateIndexes[ss]][actionIndexes[a]][endStateIndexes[es]] = rewardValue;
		  }
		}
	      }
	      
	    }
	  }
	}
      }
      
      textFile2.close();
    } else {
      cout << "Unable to open file '" << pomdpFile << "'." << endl;
      exit(1);
    }
    
    POMDP::Model<MDP::Model> model(observations.size(), states.size(), actions.size());
    
    model.setDiscount(discount);
    model.setTransitionFunction(transitionProbabilities);
    model.setObservationFunction(observationProbabilities);
    model.setRewardFunction(rewards);
    
    return model;
}

double retrieveValue(string token) {
  vector<string> tokens = parseToken(token);
  double value = 0.0;
  
  if (tokens.size() == 2) {
    value = stod(tokens[1]);
  } else {
    throw std::invalid_argument("Invalid reward value " + token + ".");
  }
  
  return value;
}

vector<int> retrieveIndexes(string token, vector<string> names) {
  int numNames = names.size();
  vector<int> indexes;
  trim(token);
  
  if (token.compare("*") == 0) {
    for (int i = 0; i < numNames; i++) {
      indexes.push_back(i);
    }
  } else {
    int index = indexOf(names, token);
    
    if ((index < 0) || (index >= numNames)) {
      throw std::invalid_argument("Action [" + token + "] not mapped.");
    }
    
    indexes.push_back(index);
  }
  
  return indexes;
}

double **buildMatrix(int numStates, string firstData, ifstream& textFile) {
  double **matrix;
  string lineText = firstData;
  int lineNumber = -1;
  
  matrix = new double*[numStates];
  
  do {
    trim(lineText);
    
    if (lineText.size() > 0) {
      vector<string> states = parseToken(lineText);
      
      if ((int)states.size() == numStates) {
	matrix[++lineNumber] = new double[numStates];
	
	for (size_t i = 0; i < states.size(); i++) {
	  matrix[lineNumber][i] = stod(states[i]);
	}
      } else {
	throw std::invalid_argument("Invalid amount of values of the line " + lineText + ".");
      }
    }
  } while (getline(textFile,lineText) && (lineNumber < (numStates - 1)));
  
  return matrix;
}

double **buildUniformMatrix(int numLines, int numCols) {
  double **matrix;
  
  matrix = new double*[numLines];
  
  for (int i = 0; i < numLines; i++) {
    matrix[i] = new double[numCols];
    for (int j = 0; j < numCols; j++) {
      matrix[i][j] = 1.0 / numCols;
    }
  }
  
  return matrix;
}

double **buildIndetityMatrix(int numLines, int numCols) {
  double **matrix;
  
  matrix = new double*[numLines];
  
  for (int i = 0; i < numLines; i++) {
    matrix[i] = new double[numCols];
    for (int j = 0; j < numCols; j++) {
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
    
    if (value.size() > 0) {
      states.push_back(value);
    }
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
    ("output-file", value<string>(&outputFile)->default_value("out.alpha"), "Use policyFileName as the name of policy output file. The file name is 'out.alpha' by default.")
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

void printModel(POMDP::Model<MDP::Model> model) {
  Matrix3D transitions = model.getTransitionFunction();
  Matrix3D observations = model.getObservationFunction();
  
  size_t numStates = model.getS();
  size_t numActions = model.getA();
  size_t numObservations = model.getO();
  
  // Print Discount
  cout << "Discount: " << model.getDiscount() << endl << endl;
  
  // Print Transitions
  for (size_t a = 0; a < numActions; a++) {
    cout << "Actions = " << a << endl;
    for (size_t ss = 0; ss < numStates; ss++) {
      for (size_t es = 0; es < numStates; es++) {
	cout << transitions[a](ss, es) << " ";
      }
      cout << endl;
    }
  }
  cout << endl;

  // Print Observations
  for (size_t a = 0; a < numActions; a++) {
    cout << "Observation = " << a << endl;
    for (size_t s = 0; s < numStates; s++) {
      for (size_t o = 0; o < numObservations; o++) {
	cout << observations[a](s, o) << " ";
      }
      cout << endl;
    }
  }
  cout << endl;
  
  // Print Rewards
  for (size_t a = 0; a < numActions; a++) {
    for (size_t ss = 0; ss < numStates; ss++) {
      for (size_t es = 0; es < numStates; es++) {
	cout << "Reward [action:" << a << "][start-state:" << ss << "][end-state:" << es << "] = " << model.getExpectedReward(ss, a, es) << endl;
      }
    }
  }
  cout << endl;
}
