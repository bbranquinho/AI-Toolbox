#include <iostream>

#include <boost/program_options.hpp>

using namespace boost::program_options;
using namespace std;

int main(int argc, char **argv) {
    // Declare the supported options.
    options_description desc("Allowed options");
    positional_options_description p;
    p.add("pomdp-file", -1);
    
    // Configuring options.
    desc.add_options()
      ("help", "Display this help message and exit.")
      ("output", value<string>()->default_value("out.policy"), "Use policyFileName as the name of policy output file. The file name is 'out.policy' by default.")
      ("pomdp-file", value<string>()->required(), "POMDP file with a pomdp model.");

    variables_map vm;
    store(command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    
    if (vm.count("help")) {
	cout << desc << "\n";
	return 1;
    }
    
    notify(vm);
    
    return 0;
}

