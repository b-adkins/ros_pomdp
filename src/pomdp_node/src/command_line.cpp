#include "command_line.h"

namespace pomdp
{

/**
 * Parses command line arguments in this library's format. Will exit if help is
 * called.
 *
 * @param[in] argc
 * @param[in] argv
 * @param[out] mdp_paths POMDP file paths read from arguments.
 * @param[out] policy_paths Policy file paths read from flags.
 * @return True on success.
 *
 * @todo Parse flags as well, perhaps using optional arguments.
 */
bool parseArgs(int argc, char* argv[],
               std::vector<std::string>& mdp_paths, std::vector<std::string>& policy_paths)
{
  std::vector<std::string> learning_paths; // Thrown away.
  return parseArgs(argc, argv, mdp_paths, policy_paths, learning_paths);
}

/**
 * Parses command line arguments in this library's format. Will exit if help is
 * called.
 *
 * @param[in] argc
 * @param[in] argv
 * @param[out] mdp_paths POMDP config file paths read from arguments.
 * @param[out] policy_paths Policy config file paths read from flags.
 * @param[out] learning_paths Learning config file paths read from flags.
 * @return True on success.
 *
 * @todo Parse flags as well, perhaps using optional arguments.
 */
bool parseArgs(int argc, char* argv[],
               std::vector<std::string>& mdp_paths, std::vector<std::string>& policy_paths,
               std::vector<std::string>& learning_paths)
{
  // Flags (e.g. --help or -h)
  namespace bargs = boost::program_options;
  std::string usage = "USAGE: " + boost::filesystem::path(argv[0]).stem().string()
                       + " POMDP_FILE...\nAllowed options";
  bargs::options_description op_desc(usage.c_str());
  op_desc.add_options()
      ("help,h", "Display this help message")
      ("POMDP_FILE", bargs::value<std::vector<std::string> >(&mdp_paths), "Path to each POMDP config file.")
      ("policy,p", bargs::value<std::vector<std::string> >(&policy_paths), "Path to each policy config file.")
      ("learn,l", bargs::value<std::vector<std::string> >(&learning_paths), "Path to each learning config file.")
    ;

  // Positional arguments
  bargs::positional_options_description arg_desc;
  arg_desc.add("POMDP_FILE", -1);

  bargs::variables_map args_var_map;
  try
  {
    bargs::store(bargs::command_line_parser(argc, argv).options(op_desc)
                   .positional(arg_desc).run(),
                 args_var_map);
    bargs::notify(args_var_map);

    if(args_var_map.count("help") || argc == 1)
    {
      std::cout << op_desc << std::endl;
      exit(EXIT_SUCCESS);
    }
  }
  catch(bargs::error& err)
  {
    std::cerr << "ERROR: " << err.what() << std::endl << std::endl;
    std::cerr << op_desc << std::endl;
    return false;
  }

  return true;
}

/**
 * Loads POMDPs from a list of config files.
 *
 * @param[in] paths Vector of config file paths.
 * @param[out] mdps Vector of pointers to initialized MDP objects. Undefined on failure.
 * @return True on success.
 */
bool loadPOMDPs(const std::vector<std::string>& paths, std::vector<MDP*>& mdps)
{
  for(int i = 0; i < paths.size(); i++)
  {
    const std::string& mdp_path = paths[i];
    std::string stem = boost::filesystem::path(mdp_path).stem().string();

    ROS_INFO("Reading POMDP file '%s'...", mdp_path.c_str());
    POMDP* pomdp = new POMDP();
    if(!pomdp->loadFromFile(mdp_path))
    {
      return false;
    }
    pomdp->setName(stem);
    mdps.push_back(pomdp);
    ROS_INFO("...as POMDP named '%s'", pomdp->getName().c_str());

    // Print initial belief state.
    ROS_INFO_STREAM("Initial state: " << pomdp->prettyState());
  }

  return true;
}

} /* namespace pomdp */
