#include <moveit/planning_interface/planning_interface.h>
#include <random>

MOVEIT_CLASS_FORWARD( ASBRContext );

class ASBRContext : public planning_interface::PlanningContext {

public:

  // TODO
  typedef std::vector<double> vertex;
  typedef std::size_t index;
  typedef std::pair<vertex,vertex> edge;
  typedef std::vector<vertex> path;

  /* Self-Defined Node Data Structure That points to its parent node and a vector 
  of node pointers that represent its children. */
  struct Node {

   public:

      vertex data;
      Node* parent;
      std::vector<Node*> children; 

      Node(const vertex& data, Node* parent)
      {
         this->data = data;
         this->parent = parent;
      }

   };

  ASBRContext( const moveit::core::RobotModelConstPtr& model,
               const std::string &name,
               const std::string& group );

  virtual ~ASBRContext();

  virtual bool solve( planning_interface::MotionPlanResponse &res );
  virtual bool solve( planning_interface::MotionPlanDetailedResponse &res );

  virtual void clear();
  virtual bool terminate();

  /**
     Test if a state collides with the scene.
     Call this method if you need to find if the robot collides with the 
     environment in the given robot's state.
     \param[in] q The robot state
     \return      true if the robot state collides. false otherwise
  */
  bool state_collides( const vertex& q ) const;

  /**
   

     Linearly interpolate between qA and qB. t is between [0,1]. If t=0, 
     it returns qA. If t=1, it returns qB. 
     \param[in] qA   The start robot configuration
     \param[in] qB   The final robot configuration
     \param     t    The joint step used in between configurations
     \return         The interpolated configuration.
  */
  vertex interpolate( const vertex& qA, const vertex& qB, double t )const;

  /**
     TODO
     
     Create a collision-free random configuration (i.e. 6 joints) of the robot.
     \param q_goal The goal configuration (use this with a bias)
     \return  A collision-free random configuration
  */
  vertex random_sample( const vertex& q_goal ) const;

  /**
     TODO

     Calculate the distance between two configurations.
     \param q1 The first configuration
     \param q2 The second configuration
     \return   The distance between q1 and q2
  */
  double distance( const vertex& q1, const vertex& q2 );

  /**
     TODO

     Search the tree for the nearest configuration to q_rand and return it.
     \param q_rand The configuration to search for a nearest neighbor.
     \return    The nearest configuration to q_rand
  */
  Node* nearest_configuration( const vertex& q_rand, const std::string& tree_type );

  /**
     TODO

     Check for collisions on a straight line path between q_near and q_new.
     \param q_near The nearest configuration in the tree
     \param q_new  The new configuration to add to the tree
     \return       True if the path is collision free. False otherwise.
  */
  bool is_subpath_collision_free( const vertex& q_near, const vertex& q_new, const double& iterations );

  
  /**
     TODO
     
     Extract a path from the tree beteen q_init and q_goal
     \param     q_init     The start configuration
     \param     q_target     The goal configuration
     \return               The path between q_init and q_goal
  */
  path search_path( const vertex& q_init, const vertex& q_target );

  /**
     SELF-DEFINED Method: Gets a q_new vertex, using an input q_rand and q_near vertex.
     
  */
  vertex get_q_new( const vertex& q_rand, const vertex& q_near, const double& step_size, const double& iterations);

  /**
     SELF-DEFINED Method: Searches one of the RRT trees for a node that has a collision-free
     straight path with an input vertex q_new. 
     
  */
  Node* returnConnected( const vertex& q_new, Node* root, const double& iterations );

  /**
     TODO

     Build a RRT tree from q_init until q_goal is reach (or give up)
     and return a path if one is found.
     \param     q_init     The start configuration
     \param     q_goal     The goal configuration
     \return               The path between q_init and q_goal
  */
  path rrt( const vertex& q_init, const vertex& q_goal );
  
protected:

  moveit::core::RobotModelConstPtr robotmodel;

  /* Self-Defined N-Ary Tree Data Structure, which contains one member variable that is a pointer
  to a root node. This root node is used to traverse the entire tree. */
  
  struct NAry_Tree {

      Node* root;

   public:

      NAry_Tree(const vertex& root) 
      {
         this->root = new Node(root, nullptr);
      }

      double distanceBetweenConfigs( const vertex& q1, const vertex& q2 );
      Node* findNearestConfig(Node* startNode, const vertex& q_rand);
      Node* findConfig(Node* root, const vertex& q_target);
      ASBRContext::path reconstructPath(const Node* config);

   };

  NAry_Tree* qinit_tree; // Pointer to a RRT rooted at q_init.
  NAry_Tree* qgoal_tree; // Pointer to a RRT rooted at q_goal.

};

