/**
 * Author: Blaine Oania
 * Filename: goal_controller.h
 * Date: 12/13/22
 * Package: first_bts
 * Description:
 *  Handles retrieving, passing, and selection of goals. This implementation
 *  is completely divorced from ROS, aside from handling errors and reading package
 *  directory. 
*/

#include "goal_controller.h"

BT::NodeStatus GoalController::ReadGoals::tick() {
    std::string path = ros::package::getPath("first_bts").append("/src/");
    std::string file_name;
    std::ifstream file;
    std::string line;
    std::string goals;
    double num_goals = 0;

    BT::Optional<std::string> opt_file_name = getInput<std::string>("file");
    if(!opt_file_name)
        throw BT::RuntimeError("Error reading file name: ", opt_file_name.error());
    file_name = opt_file_name.value();
    path.append(file_name);

    // Fail node if file does not open
    file.open(path, std::ios::in);
    if(!file.is_open())
        return BT::NodeStatus::FAILURE;
    
    // Concatenate all goals into one stirng
    while( getline(file, line))
    {
        goals.append(line);
        goals.append("/");
        num_goals++;
    }

    file.close();

    // Set output port values
    setOutput("goals", goals);
    setOutput("num_goals", num_goals);

    // std::cout << "Goals: " << goals << std::endl;

    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GoalController::RequestGoal::tick() {
    std::string goals;
    std::string goal;
    double goal_count;
    double index;


    BT::Optional<std::string> opt_goals = getInput<std::string>("goals");
    if(!opt_goals)
        throw BT::RuntimeError("Error reading goal list: ", opt_goals.error());
    goals = opt_goals.value();

    BT::Optional<double> opt_index = getInput<double>("index");
    if(!opt_index)
        throw BT::RuntimeError("Error reading desired index: ", opt_index.error());
    index = opt_index.value();

    BT::Optional<double> opt_goal_count = getInput<double>("num_goals");
    if(!opt_goal_count)
        throw BT::RuntimeError("Error reading desired index: ", opt_index.error());
    goal_count = opt_goal_count.value();

    // Fail node if no more goals
    if(index >= goal_count)
    {
        ROS_ERROR("Goal index is out of range.");
        return BT::NodeStatus::FAILURE;
    }
    
    // There must be a better way to do this
    // Step through string until start of desired goal
    std::size_t i = 0;
    std::size_t count = 0;
    while(count != index)
    {
        if(goals.at(i) == '/')
            count++;
        
        i++;
    }

    // Pull goal information out
    while(goals.at(i) != '/')
    {
        goal.append(std::string(1, goals.at(i)));
        i++;
    }

    // Set output port values
    setOutput("goal", goal);

    return BT::NodeStatus::SUCCESS;
    
}

// Simply increment goal index by 1
BT::NodeStatus GoalController::IncrementIndex::tick() {
    double index;

    BT::Optional<double> opt_index = getInput<double>("index");
    if(!opt_index)
        throw BT::RuntimeError("Error reading desired index: ", opt_index.error());
    index = opt_index.value();

    index = index + 1;

    setOutput("index_out", index);

    std::cout << "index: " << index << std::endl;

    return BT::NodeStatus::SUCCESS;
}