#include "assignment3_context.h"
#include <moveit/planning_scene/planning_scene.h>
/* Additional Imports/Includes */
#include<queue>
#include <algorithm>

/* Instructor-Provided Method */
ASBRContext::ASBRContext( const moveit::core::RobotModelConstPtr& robotmodel,
                          const std::string& name,
                          const std::string& group ):
  planning_interface::PlanningContext( name, group ),
  robotmodel( robotmodel ){}

ASBRContext::~ASBRContext(){}
/* Instructor-Provided Method */
bool ASBRContext::state_collides( const vertex& q ) const {

  // create a robot state for the UR5
  moveit::core::RobotState robotstate( robotmodel );
  robotstate.setJointGroupPositions( "ur5e", q );

  // check for a collision for the UR5+gripper
  if( getPlanningScene()->isStateColliding( robotstate, "manipulator", false ) )
    { return true; }
  else
    { return false; }

}

/* Instructor-Provided Method */
ASBRContext::vertex ASBRContext::interpolate( const ASBRContext::vertex& qA,
                                              const ASBRContext::vertex& qB,
                                              double t ) const {

  ASBRContext::vertex qt( qA.size(), 0.0 );
  for( std::size_t i=0; i<qt.size(); i++ )
    { qt[i] = ( 1.0 - t )*qA[i] + t*qB[i]; }

  return qt;

}

/* Self-Written Method */
// TODO
ASBRContext::vertex ASBRContext::random_sample( const ASBRContext::vertex& q_goal ) const {  
  ASBRContext::vertex q_rand;
  // TODO return a random sample q_rand with goal bias.

  std::random_device rd; 
  std::mt19937 gen(rd()); 
  std::uniform_int_distribution<> dist_i(0, 99); 
  
  // Goal Bias: Generate a random number from 0 to 99. Each number has an equal chance 
  // to be generated. 
  int rand = dist_i(gen);
  if (rand > 94) {
    return q_goal;
  }

  std::uniform_real_distribution<double> dist_d(-M_PI, M_PI); 

  // Generate 6 random doubles between -pi and pi.
  for (int i = 0; i < 6; ++i) {
    double randq = dist_d(gen);
    q_rand.push_back(randq);
  }

  // if the above generated q_rand collides, continously generate new q_rands until 
  // there is one that doesn't. 
  while (state_collides(q_rand)) {
    q_rand.clear();
    for (int i = 0; i < 6; ++i) {
      double randq = dist_d(gen);
      q_rand.push_back(randq);
    }
  }
  return q_rand;
}


/* Self-Written Method */
double ASBRContext::distance( const ASBRContext::vertex& q1, const ASBRContext::vertex& q2 ){
  double d=0;
  // TODO compute a distance between two configurations (your choice of metric).

  /* 
  Note: This method was unused because a method belonging to the NAry-Tree class needed to 
  use a similar method, so I defined a new NAry-Tree method that is identical to this one. 
  This method is just here because the assignment required implementation. 
  */ 

  // Euclidean Metric (Squared differences between q1 and q2)
  for (int i = 0; i < 6; i++) {

    double diff = q2[i]-q1[i];
    double diff2 = diff*diff;
    d += diff2;

  }

  d = sqrt(d);

  return d;
}

/* Self-Written Method */
ASBRContext::Node* ASBRContext::nearest_configuration( const ASBRContext::vertex& q_rand, const std::string& tree_type ){

  // TODO find the nearest configuration in the tree to q_rand

  /*
  Note: Like the distance method, this method was unused because it was easier to define a member 
  method for NAry-Tree to do the same thing. If I were to use it, it would just call the method in NAry-Tree
  called findNearestConfig
  */

  ASBRContext::Node* startNode;
  ASBRContext::Node* nearest;

  if (tree_type == "qinit") {
    startNode = qinit_tree->root;
  }
  if (tree_type == "qgoal") {
    startNode = qgoal_tree->root;
  }
  
  nearest = qinit_tree->findNearestConfig(startNode, q_rand);

  return nearest;
}

/* Self-Written Method */
bool ASBRContext::is_subpath_collision_free( const ASBRContext::vertex& q_near,
					      const ASBRContext::vertex& q_new, const double& iterations ){
  // TODO find if the straightline path between q_near and q_rand is collision free

  for (double i = 0.0; i < iterations; i++) { // Iterations define how much 
                                              // to discretize the path into. 
    double t = i/iterations; // Get the interpolation proportion.
    ASBRContext::vertex q_interp = interpolate(q_near, q_new, t);
    if (state_collides(q_interp)) {
      return false;
    }

  }
  return true;
}


/* Self-Written Method */
ASBRContext::path ASBRContext::search_path( const vertex& q_init,
					    const ASBRContext::vertex& q_target ){
  ASBRContext::path P;
  // TODO Once q_goal has been added to the tree, find the path (sequence of configurations) between
  // q_init and q_goal (hint: this is easier by using recursion).

  /* 
    Note: This method also ended up being unused. It was easier to define a method in NAry-Tree 
    that searches for a path from the root to the goal through recursion. 
  */

  ASBRContext::Node* rootNode;
  ASBRContext::Node* targetNode;

  if (qinit_tree->root->data == q_init) {
    rootNode = qinit_tree->root;
  }
  else {
    rootNode = qgoal_tree->root;
  }

  targetNode = qinit_tree->findConfig(rootNode, q_target);
  P = qinit_tree->reconstructPath(targetNode);

  return P;
}

/* Self-Written Method */
ASBRContext::vertex ASBRContext::get_q_new( const ASBRContext::vertex& q_rand, 
                                           const ASBRContext::vertex& q_near, 
                                           const double& step_size, const double& iterations ) {

  // Progress along the straightline path from q_near to q_rand by some step. This step_size 
  // is a proportion. It is used to get a q_new that is located on the straightline path
  // at a distance which is the step_size multiplied by the total_distance between q_near and q_rand.                                     
  ASBRContext::vertex q_new = interpolate(q_near, q_rand, step_size);

  if (is_subpath_collision_free(q_near, q_new, iterations)) {
    return q_new;
  } else {
    q_new.clear();
    q_new.push_back(1.0);
  }

  return q_new;
}

/* Self-Written Method */
ASBRContext::Node* ASBRContext::returnConnected( const ASBRContext::vertex& q_new, ASBRContext::Node* root, const double& iterations ) {
  // BFS Method to search the entire tree. 
  if (is_subpath_collision_free(root->data, q_new, iterations)) {
    return root;
  }

  // Queue Initialization
  std::queue<ASBRContext::Node*> bfs_queue;

  for (ASBRContext::Node* child: root->children) {
    bfs_queue.push(child);
  }

  // While the Queue isn't empty, keep getting the front of the queue and checking if it can connect
  // to the q_new vertex. 
  while (!bfs_queue.empty()) {

    ASBRContext::Node* frontNode = bfs_queue.front();
    bfs_queue.pop();

    if ((is_subpath_collision_free(frontNode->data, q_new, iterations))) {
      return frontNode;
    }

    // Push the children of the current node being examined to the back of the queue.
    for (ASBRContext::Node* child: frontNode->children) {
      bfs_queue.push(child);
    } 

  }

  return nullptr;
}

/* Self-Written Method */
/* The Most Important Function: The Main RRT Method for Path Planning */
ASBRContext::path ASBRContext::rrt( const ASBRContext::vertex& q_init,
				    const ASBRContext::vertex& q_goal ){
  ASBRContext::path P;

  ASBRContext::vertex q_init_cut = q_init;
  ASBRContext::vertex q_goal_cut = q_goal;
  q_init_cut.resize(6);
  q_goal_cut.resize(6);

  // Instantiate Two Trees, one rooted at q_init, and one rooted at q_goal.
  qinit_tree = new NAry_Tree(q_init_cut);
  qgoal_tree = new NAry_Tree(q_goal_cut);

  // Control the amount of times the algorithm attempts to find a path.
  int limit = 100000;
  int attempts = 0;

  // Step size that progresses q_near along the straight line between q_near and q_rand. 
  // It is defined as a proportion of the straight-line distance between the configurations.
  // Iterations defines how much to discretize the straightline path between two configs, when
  // checking for collisions.
  double step_size = 0.1;
  double iterations = 40.0;

  // Use this to track which tree should be the one being expanded at the current 
  // iteration of the while loop.
  bool switchTree = false;

  // root will always be the root of the tree being expanded in the current iteration
  // of the loop. other_root will always be the root of the other tree. 
  ASBRContext::Node* root_node;
  ASBRContext::Node* other_root;


  while (attempts < limit) {

    if (switchTree) {
      root_node = qgoal_tree->root;
      other_root = qinit_tree->root;
    } else {
      root_node = qinit_tree->root;
      other_root = qgoal_tree->root;
    }

    // Get Random Collision-Free Vertex With Goal Bias
    ASBRContext::vertex q_rand = random_sample(other_root->data);

    // Search the current tree of interest for a node containing a configuration that 
    // has the nearest distance to q_rand, out of all other nodes.
    ASBRContext::Node* q_near_node = qinit_tree->findNearestConfig(root_node, q_rand);
    ASBRContext::vertex q_near = q_near_node->data;

    // Get a q_new, which is q_near progressed by a step_size towards q_rand
    ASBRContext::vertex q_new = get_q_new(q_rand, q_near, step_size, iterations);

    if (q_new.size() != 1) { // If you have got a successful q_new configuration

      // Instantiate a new Node that contains the vertex q_new, and its parent is q_near_node.
      ASBRContext::Node* q_new_node = new Node(q_new, q_near_node);

      // Add the q_new_node to the children of q_near_node, adding it to the tree.
      q_near_node->children.push_back(q_new_node);

      // Check if there is any node in the other tree that will have a collision-free path with
      // this q_new configuration.
      ASBRContext::Node* otherSuccess = returnConnected(q_new, other_root, iterations);

      if (otherSuccess != nullptr) { // If you have successfully found a node in the other tree that 
                                     // has a collision-free path with q_new

          ASBRContext::path P1; // Initialize path originating from q_init.
          ASBRContext::path P2; // Initialize path originating from q_goal.

          if (switchTree) {

            P1 = qinit_tree->reconstructPath(otherSuccess); // Get the Path originating from q_init
            P2 = qinit_tree->reconstructPath(q_new_node);   // Get the Path originating from q_goal

          } else {

            P2 = qinit_tree->reconstructPath(otherSuccess);
            P1 = qinit_tree->reconstructPath(q_new_node);

          }

          std::reverse(P2.begin(), P2.end()); // Reverse P2 (Normally, it begins with q_goal)
          P1.insert(P1.end(), P2.begin(), P2.end()); // Concatenate P2-reversed to the end of P1 to get 
                                                     // the full path.
          return P1;

      }

    }
    switchTree = !switchTree;
    attempts++;
  }
  return P;
}

/* Instructor-Provided Method */
bool ASBRContext::solve( planning_interface::MotionPlanResponse &res ){

  // This is the method that is called each time a plan is requested
  // Create a new empty trajectory
  res.trajectory_.reset(new robot_trajectory::RobotTrajectory(robotmodel,getGroupName()));
  res.trajectory_->clear();
  
  // copy the initial/final joints configurations to vectors qfin and qinit
  // This is mainly for convenience.
  std::vector<double> qstart, qfinal;

  for( size_t i=0; i<robotmodel->getVariableCount(); i++ ){
    qfinal.push_back(request_.goal_constraints[0].joint_constraints[i].position);
    qstart.push_back(request_.start_state.joint_state.position[i]);
  }
  
  // start the timer
  rclcpp::Clock clock;
  rclcpp::Time t1 = clock.now();
  path P = rrt( qstart, qfinal );
  rclcpp::Time t2 = clock.now();
  std::cout << "Your path has length " << P.size() << std::endl;
  // end the timer
  
  // The rest is to fill in the animation.
  moveit::core::RobotState robotstate( robotmodel );
  robotstate.setJointGroupPositions( "ur5e", qstart );
  res.trajectory_->addSuffixWayPoint( robotstate, 0.2 );

  for( std::size_t i=1; i<P.size(); i++ ){
    for( double t=0.0; t<=1.0; t+=0.01 ){
      vertex q = interpolate( P[i-1], P[i], t );
      robotstate.setJointGroupPositions( "ur5e", q );
      res.trajectory_->addSuffixWayPoint( robotstate, 0.2 );
    }
  }

  // 
  rclcpp::Duration planning_time = t2-t1;
  res.planning_time_ = planning_time.seconds();
  res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  
  return true;
  
}

/* Instructor-Provided Method */
bool ASBRContext::solve( planning_interface::MotionPlanDetailedResponse& )
{ return true; }

void ASBRContext::clear(){}

bool ASBRContext::terminate(){return true;}


/* Self-Defined Data Structure Methods */

/* Self-Written Method */
double ASBRContext::NAry_Tree::distanceBetweenConfigs( const ASBRContext::vertex& q1, const ASBRContext::vertex& q2 ) {

  double d=0;
  // Identical to distance method in ASBRContext. 

  // Euclidean Metric (Squared differences between q1 and q2)
  for (int i = 0; i < 6; i++) {
    double diff = q2[i]-q1[i];
    double diff2 = diff*diff;
    d += diff2;
  }

  return sqrt(d);

}

/* Self-Written Method */
// Breadth-First Search on Tree to Find the Node with the Nearest Distance to q_rand
ASBRContext::Node* ASBRContext::NAry_Tree::findNearestConfig(ASBRContext::Node* startNode, const ASBRContext::vertex& q_rand) {

  // Nearest Configuration and Nearest Distance Initialization
  ASBRContext::Node* nearestConfig = startNode;
  double nearestDist = distanceBetweenConfigs(startNode->data, q_rand);

  // Queue Initialization
  std::queue<ASBRContext::Node*> bfs_queue;
  for (ASBRContext::Node* child: startNode->children) {
    bfs_queue.push(child);
  }

  while (!bfs_queue.empty()) {

    // In a while loop, while the queue is not empty, keep examining the front of the queue. 
    // Update the nearest distance, if the front of the queue has the shortest distance to q_rand. 
    // Then, pop the front node from the queue, and push its children to the back of the queue. 

    ASBRContext::Node* frontNode = bfs_queue.front();
    bfs_queue.pop();

    double currentDist = distanceBetweenConfigs(frontNode->data, q_rand);

    if (currentDist < nearestDist) {
      nearestDist = currentDist;
      nearestConfig = frontNode;
    }

    for (ASBRContext::Node* child: frontNode->children) {
      bfs_queue.push(child);
    } 

  }

  return nearestConfig;
}

/* Self-Written Method */
ASBRContext::Node* ASBRContext::NAry_Tree::findConfig(ASBRContext::Node* root, const ASBRContext::vertex& q_target) {

  if (root->data == q_target) {
    return root;
  }

  // Queue Initialization
  std::queue<ASBRContext::Node*> bfs_queue;
  for (ASBRContext::Node* child: root->children) {
    bfs_queue.push(child);
  }

  while (!bfs_queue.empty()) {

    ASBRContext::Node* frontNode = bfs_queue.front();
    bfs_queue.pop();

    if (frontNode->data == q_target) {
      return frontNode;
    }

    for (ASBRContext::Node* child: frontNode->children) {
      bfs_queue.push(child);
    } 

  }

  return nullptr;

}

/* Self-Written Method */
// Recursively gets the path from a tree's root to any configuration in the tree, defined by config.
ASBRContext::path ASBRContext::NAry_Tree::reconstructPath(const ASBRContext::Node* config) {

  // Base Case: If you reach the root node, return a path datatype that has only 
  // the root node in its vector. 

  // Initialize a path datatype. Only place the input parameter config in this path datatype. 

  // Recursive Case: 
  // If the node currently being examined has a parent, 
      // Set a new path datatype equal to the call of reconstructPath on this node's parent
      // Add the result of this recursive call to the path datatype initialized before the recursive case. 

  // Return that initial path datatype. 

  ASBRContext::path P;
  P.push_back(config->data);
  ASBRContext::path reconstructedPath;

  if (config->parent == nullptr) {

    return P;

  } else {

    reconstructedPath = reconstructPath(config->parent);
    reconstructedPath.insert(reconstructedPath.end(), P.begin(), P.end());

  }

  return reconstructedPath;
}