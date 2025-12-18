// src/lib/content-recommendations.ts
// Content recommendation engine based on user background questionnaire

export interface RecommendedContent {
  title: string;
  path: string;
  excerpt: string;
  reason: string;
  priority: number; // 1-5, 5 being highest priority
  category: 'getting-started' | 'fundamentals' | 'advanced' | 'practical';
}

export interface UserResponses {
  experience_level?: string[];
  programming_languages?: string[];
  robotics_topics?: string[];
  humanoid_interest?: string[];
  learning_goals?: string[];
  prior_platforms?: string[];
}

/**
 * Generate personalized content recommendations based on user questionnaire responses
 */
export function getRecommendedContent(
  softwareBackground: any,
  hardwareBackground: any
): RecommendedContent[] {
  const recommendations: RecommendedContent[] = [];

  // Combine all responses
  const responses: UserResponses = {
    ...softwareBackground,
    ...hardwareBackground
  };

  // Helper function to check if user selected an option
  const hasResponse = (questionId: keyof UserResponses, value: string): boolean => {
    const response = responses[questionId];
    if (Array.isArray(response)) {
      return response.includes(value);
    }
    return response === value;
  };

  // Helper to check if any option is selected
  const hasAnyResponse = (questionId: keyof UserResponses, values: string[]): boolean => {
    return values.some(value => hasResponse(questionId, value));
  };

  // Recommendations for Complete Beginners
  if (hasResponse('experience_level', 'Complete Beginner') ||
      hasResponse('programming_languages', 'None yet')) {
    recommendations.push({
      title: 'Getting Started with Robotics',
      path: '/docs/intro',
      excerpt: 'Begin your robotics journey with fundamental concepts and prerequisites.',
      reason: 'Perfect starting point for beginners',
      priority: 5,
      category: 'getting-started'
    });
    recommendations.push({
      title: 'Introduction to Programming for Robotics',
      path: '/docs/fundamentals/programming-basics',
      excerpt: 'Learn programming fundamentals needed for robotics development.',
      reason: 'Essential for those new to programming',
      priority: 5,
      category: 'getting-started'
    });
  }

  // Recommendations based on interest in Kinematics
  if (hasResponse('robotics_topics', 'Kinematics & Motion Planning')) {
    recommendations.push({
      title: 'Forward and Inverse Kinematics',
      path: '/docs/kinematics/forward-inverse',
      excerpt: 'Master the mathematics of robot motion and positioning.',
      reason: 'Matches your interest in kinematics',
      priority: 4,
      category: 'fundamentals'
    });
    recommendations.push({
      title: 'Motion Planning Algorithms',
      path: '/docs/motion-planning/algorithms',
      excerpt: 'Learn path planning and trajectory optimization techniques.',
      reason: 'Advanced motion planning techniques',
      priority: 3,
      category: 'advanced'
    });
  }

  // Recommendations for Computer Vision interest
  if (hasResponse('robotics_topics', 'Computer Vision')) {
    recommendations.push({
      title: 'Vision Systems for Humanoid Robots',
      path: '/docs/perception/computer-vision',
      excerpt: 'Implement visual perception for humanoid robots.',
      reason: 'Aligns with your computer vision interest',
      priority: 4,
      category: 'fundamentals'
    });
    recommendations.push({
      title: 'Object Detection and Recognition',
      path: '/docs/perception/object-detection',
      excerpt: 'Real-time object detection for robot manipulation.',
      reason: 'Practical computer vision applications',
      priority: 3,
      category: 'practical'
    });
  }

  // Recommendations for Machine Learning & AI
  if (hasResponse('robotics_topics', 'Machine Learning & AI')) {
    recommendations.push({
      title: 'Machine Learning for Robotics',
      path: '/docs/ai/machine-learning',
      excerpt: 'Apply ML techniques to improve robot behavior.',
      reason: 'Matches your AI/ML interest',
      priority: 4,
      category: 'advanced'
    });
    recommendations.push({
      title: 'Reinforcement Learning for Locomotion',
      path: '/docs/ai/reinforcement-learning',
      excerpt: 'Train humanoid robots to walk using RL.',
      reason: 'Cutting-edge AI techniques',
      priority: 3,
      category: 'advanced'
    });
  }

  // Recommendations for Control Systems
  if (hasResponse('robotics_topics', 'Control Systems')) {
    recommendations.push({
      title: 'Control Theory Fundamentals',
      path: '/docs/control/fundamentals',
      excerpt: 'PID controllers and feedback systems for robots.',
      reason: 'Essential for control systems',
      priority: 4,
      category: 'fundamentals'
    });
    recommendations.push({
      title: 'Balance Control for Bipedal Robots',
      path: '/docs/control/balance',
      excerpt: 'Advanced techniques for maintaining robot stability.',
      reason: 'Advanced control applications',
      priority: 3,
      category: 'advanced'
    });
  }

  // Recommendations for ROS/ROS2 interest
  if (hasResponse('robotics_topics', 'ROS/ROS2') ||
      hasResponse('prior_platforms', 'ROS/ROS2')) {
    recommendations.push({
      title: 'ROS 2 for Humanoid Robots',
      path: '/docs/software/ros2-intro',
      excerpt: 'Set up ROS 2 for humanoid robot development.',
      reason: 'Build on your ROS experience',
      priority: 5,
      category: 'practical'
    });
    recommendations.push({
      title: 'Creating Custom ROS 2 Nodes',
      path: '/docs/software/ros2-nodes',
      excerpt: 'Develop custom nodes for humanoid robot control.',
      reason: 'Practical ROS development',
      priority: 4,
      category: 'practical'
    });
  }

  // Recommendations based on humanoid-specific interests
  if (hasResponse('humanoid_interest', 'Bipedal Walking')) {
    recommendations.push({
      title: 'Bipedal Locomotion',
      path: '/docs/locomotion/bipedal-walking',
      excerpt: 'Understand and implement bipedal walking gaits.',
      reason: 'Matches your interest in walking robots',
      priority: 5,
      category: 'fundamentals'
    });
    recommendations.push({
      title: 'Gait Generation and Optimization',
      path: '/docs/locomotion/gait-optimization',
      excerpt: 'Create efficient and stable walking patterns.',
      reason: 'Advanced walking techniques',
      priority: 4,
      category: 'advanced'
    });
  }

  if (hasResponse('humanoid_interest', 'Manipulation & Grasping')) {
    recommendations.push({
      title: 'Robotic Manipulation Fundamentals',
      path: '/docs/manipulation/fundamentals',
      excerpt: 'Learn how humanoid robots interact with objects.',
      reason: 'Focused on manipulation skills',
      priority: 5,
      category: 'fundamentals'
    });
    recommendations.push({
      title: 'Grasp Planning and Execution',
      path: '/docs/manipulation/grasping',
      excerpt: 'Advanced techniques for robust object grasping.',
      reason: 'Practical manipulation skills',
      priority: 4,
      category: 'practical'
    });
  }

  if (hasResponse('humanoid_interest', 'Human-Robot Interaction')) {
    recommendations.push({
      title: 'Human-Robot Interaction Design',
      path: '/docs/hri/design-principles',
      excerpt: 'Create intuitive interfaces for robot interaction.',
      reason: 'Aligns with HRI interest',
      priority: 4,
      category: 'fundamentals'
    });
    recommendations.push({
      title: 'Social Robotics and Communication',
      path: '/docs/hri/social-robotics',
      excerpt: 'Enable natural human-robot communication.',
      reason: 'Advanced HRI concepts',
      priority: 3,
      category: 'advanced'
    });
  }

  if (hasResponse('humanoid_interest', 'Balance & Stability')) {
    recommendations.push({
      title: 'Balance and Stability Control',
      path: '/docs/control/balance',
      excerpt: 'Techniques for maintaining robot balance.',
      reason: 'Essential for stable humanoid robots',
      priority: 5,
      category: 'fundamentals'
    });
    recommendations.push({
      title: 'Zero Moment Point (ZMP) Control',
      path: '/docs/control/zmp',
      excerpt: 'Implement ZMP-based stability control.',
      reason: 'Industry-standard stability technique',
      priority: 4,
      category: 'advanced'
    });
  }

  // Recommendations based on learning goals
  if (hasResponse('learning_goals', 'Build my own humanoid robot')) {
    recommendations.push({
      title: 'Hardware Design for Humanoid Robots',
      path: '/docs/hardware/design-guide',
      excerpt: 'Complete guide to building your own humanoid robot.',
      reason: 'Perfect for DIY robot builders',
      priority: 5,
      category: 'practical'
    });
    recommendations.push({
      title: 'Component Selection Guide',
      path: '/docs/hardware/components',
      excerpt: 'Choose the right motors, sensors, and controllers.',
      reason: 'Essential for hardware projects',
      priority: 5,
      category: 'practical'
    });
  }

  if (hasResponse('learning_goals', 'Academic research')) {
    recommendations.push({
      title: 'Research Methodologies in Humanoid Robotics',
      path: '/docs/research/methodologies',
      excerpt: 'Conduct rigorous robotics research.',
      reason: 'Tailored for researchers',
      priority: 4,
      category: 'advanced'
    });
    recommendations.push({
      title: 'State-of-the-Art Humanoid Systems',
      path: '/docs/research/state-of-art',
      excerpt: 'Survey of cutting-edge humanoid robots.',
      reason: 'Research background',
      priority: 3,
      category: 'advanced'
    });
  }

  // For users with simulation experience
  if (hasResponse('prior_platforms', 'Simulation (Gazebo, MuJoCo, etc.)')) {
    recommendations.push({
      title: 'Simulation for Humanoid Robots',
      path: '/docs/simulation/gazebo',
      excerpt: 'Test and validate your robots in simulation.',
      reason: 'Build on your simulation experience',
      priority: 4,
      category: 'practical'
    });
    recommendations.push({
      title: 'Sim-to-Real Transfer',
      path: '/docs/simulation/sim-to-real',
      excerpt: 'Bridge the gap between simulation and reality.',
      reason: 'Advanced simulation techniques',
      priority: 3,
      category: 'advanced'
    });
  }

  // Python users get Python-specific content
  if (hasResponse('programming_languages', 'Python')) {
    recommendations.push({
      title: 'Python for Robotics',
      path: '/docs/programming/python',
      excerpt: 'Leverage Python for rapid robot development.',
      reason: 'Perfect for Python developers',
      priority: 4,
      category: 'practical'
    });
  }

  // C++ users get performance-focused content
  if (hasResponse('programming_languages', 'C++')) {
    recommendations.push({
      title: 'Real-Time Control with C++',
      path: '/docs/programming/cpp-realtime',
      excerpt: 'High-performance robot control in C++.',
      reason: 'Matches your C++ expertise',
      priority: 4,
      category: 'advanced'
    });
  }

  // Sort by priority (highest first) and return top 8-10 recommendations
  return recommendations
    .sort((a, b) => b.priority - a.priority)
    .slice(0, 10);
}
