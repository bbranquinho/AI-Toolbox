#include <iostream>

#include <boost/program_options.hpp>

using namespace boost::program_options;
using namespace std;

int main(int argc, char **argv) {
    // Declare the supported options.
    options_description desc("Allowed options");
    desc.add_options()
      ("help", "Display this help message and exit.")
      ("output", value<string>(), "Use policyFileName as the name of policy output file. The file name is 'out.policy' by default.");

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);    

    if (vm.count("help")) {
	cout << desc << "\n";
	return 1;
    }

    
    
    return 0;
}
