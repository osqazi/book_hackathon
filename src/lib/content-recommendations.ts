// src/lib/content-recommendations.ts
// Content recommendation engine - ONLY recommends pages that actually exist

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
  python_experience?: string[];
  primary_interest?: string[];
}

/**
 * Generate personalized content recommendations based on user questionnaire responses
 * ONLY returns pages that actually exist in the documentation
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

  // ===== ALWAYS RECOMMEND: Introduction =====
  recommendations.push({
    title: 'Introduction to Humanoid Robotics',
    path: '/docs/intro',
    excerpt: 'Start your journey through four essential modules: ROS 2, Simulation, AI Navigation, and Vision-Language-Action systems.',
    reason: 'Essential starting point for all learners',
    priority: 5,
    category: 'getting-started'
  });

  // ===== COMPLETE BEGINNERS =====
  if (hasResponse('experience_level', 'Complete Beginner')) {
    // Start with Module 1
    recommendations.push({
      title: 'Module 1: ROS 2 Fundamentals',
      path: '/docs/module-1-ros2/',
      excerpt: 'Learn the robotic nervous system - ROS 2 architecture, communication patterns, and robot modeling.',
      reason: 'Perfect starting point for beginners',
      priority: 5,
      category: 'getting-started'
    });

    recommendations.push({
      title: 'ROS 2 Nodes and Topics',
      path: '/docs/module-1-ros2/nodes-topics',
      excerpt: 'Understanding the basic building blocks of ROS 2 communication.',
      reason: 'First concepts to learn in ROS 2',
      priority: 4,
      category: 'fundamentals'
    });
  }

  // ===== ROS 2 INTEREST =====
  if (hasResponse('primary_interest', 'ROS 2 Fundamentals') ||
      hasResponse('primary_interest', 'All modules equally')) {

    recommendations.push({
      title: 'Module 1: ROS 2 Fundamentals',
      path: '/docs/module-1-ros2/',
      excerpt: 'Learn the robotic nervous system - ROS 2 architecture, communication patterns, and robot modeling.',
      reason: 'Matches your interest in ROS 2',
      priority: 5,
      category: 'fundamentals'
    });

    recommendations.push({
      title: 'ROS 2 Nodes and Topics',
      path: '/docs/module-1-ros2/nodes-topics',
      excerpt: 'Master asynchronous communication in ROS 2 using publish-subscribe patterns.',
      reason: 'Core ROS 2 communication concepts',
      priority: 5,
      category: 'fundamentals'
    });

    recommendations.push({
      title: 'ROS 2 Services and Actions',
      path: '/docs/module-1-ros2/services-actions',
      excerpt: 'Learn synchronous request-response and long-running task patterns in ROS 2.',
      reason: 'Essential ROS 2 patterns',
      priority: 4,
      category: 'fundamentals'
    });

    recommendations.push({
      title: 'URDF Robot Modeling',
      path: '/docs/module-1-ros2/urdf-modeling',
      excerpt: 'Describe humanoid robot structure using the Unified Robot Description Format.',
      reason: 'Learn to model humanoid robots',
      priority: 4,
      category: 'practical'
    });
  }

  // ===== SIMULATION & PHYSICS INTEREST =====
  if (hasResponse('primary_interest', 'Simulation & Physics') ||
      hasResponse('primary_interest', 'All modules equally')) {

    recommendations.push({
      title: 'Module 2: Simulation & Digital Twins',
      path: '/docs/module-2-simulation/',
      excerpt: 'Master virtual environments for safe, cost-effective robot development using Gazebo and Unity.',
      reason: 'Matches your simulation interest',
      priority: 5,
      category: 'fundamentals'
    });

    recommendations.push({
      title: 'Physics Principles for Simulation',
      path: '/docs/module-2-simulation/physics-principles',
      excerpt: 'Understand the physics engines behind realistic robot simulation.',
      reason: 'Foundation of robot simulation',
      priority: 4,
      category: 'fundamentals'
    });

    recommendations.push({
      title: 'Digital Twin Workflows',
      path: '/docs/module-2-simulation/digital-twin',
      excerpt: 'Create virtual replicas of physical robots for testing before deployment.',
      reason: 'Industry-standard practice',
      priority: 4,
      category: 'practical'
    });

    recommendations.push({
      title: 'Sensor Integration in Simulation',
      path: '/docs/module-2-simulation/sensors',
      excerpt: 'Integrate LiDAR, depth cameras, IMU, and RGB sensors in virtual environments.',
      reason: 'Essential for realistic simulation',
      priority: 4,
      category: 'practical'
    });
  }

  // ===== AI NAVIGATION / ISAAC INTEREST =====
  if (hasResponse('primary_interest', 'AI Navigation (NVIDIA Isaac)') ||
      hasResponse('primary_interest', 'All modules equally')) {

    recommendations.push({
      title: 'Module 3: AI-Driven Perception (NVIDIA Isaac)',
      path: '/docs/module-3-isaac/',
      excerpt: 'Unlock photorealistic simulation and AI-powered navigation with NVIDIA Isaac.',
      reason: 'Matches your interest in AI navigation',
      priority: 5,
      category: 'advanced'
    });

    recommendations.push({
      title: 'Isaac Sim for Robotics',
      path: '/docs/module-3-isaac/isaac-sim',
      excerpt: 'Use NVIDIA Isaac Sim for photorealistic sensor simulation and AI training.',
      reason: 'State-of-the-art simulation platform',
      priority: 5,
      category: 'advanced'
    });

    recommendations.push({
      title: 'Visual SLAM Navigation',
      path: '/docs/module-3-isaac/vslam',
      excerpt: 'Enable robots to build maps and navigate using visual simultaneous localization and mapping.',
      reason: 'Critical for autonomous navigation',
      priority: 4,
      category: 'advanced'
    });

    recommendations.push({
      title: 'Nav2 Navigation Stack',
      path: '/docs/module-3-isaac/nav2',
      excerpt: 'Implement autonomous navigation using the ROS 2 Nav2 stack.',
      reason: 'Industry-standard navigation',
      priority: 4,
      category: 'practical'
    });

    recommendations.push({
      title: 'Synthetic Data Generation',
      path: '/docs/module-3-isaac/synthetic-data',
      excerpt: 'Generate training data for AI models using photorealistic simulation.',
      reason: 'Accelerate AI development',
      priority: 3,
      category: 'advanced'
    });
  }

  // ===== VISION-LANGUAGE-ACTION / LLM INTEREST =====
  if (hasResponse('primary_interest', 'Vision-Language-Action (LLM/AI)') ||
      hasResponse('primary_interest', 'All modules equally')) {

    recommendations.push({
      title: 'Module 4: Vision-Language-Action Integration',
      path: '/docs/module-4-vla/',
      excerpt: 'Build autonomous humanoid systems that understand and execute natural language commands.',
      reason: 'Matches your interest in LLM/AI',
      priority: 5,
      category: 'advanced'
    });

    recommendations.push({
      title: 'VLA Architecture Overview',
      path: '/docs/module-4-vla/architecture',
      excerpt: 'Understand how vision, language, and action work together in cognitive robots.',
      reason: 'Foundation of intelligent robotics',
      priority: 5,
      category: 'advanced'
    });

    recommendations.push({
      title: 'LLM-Based Task Planning',
      path: '/docs/module-4-vla/llm-planning',
      excerpt: 'Use large language models to plan complex robot tasks from natural language.',
      reason: 'Cutting-edge AI for robotics',
      priority: 4,
      category: 'advanced'
    });

    recommendations.push({
      title: 'Multimodal Perception',
      path: '/docs/module-4-vla/multimodal',
      excerpt: 'Combine vision, language, and sensor data for comprehensive understanding.',
      reason: 'Advanced perception techniques',
      priority: 4,
      category: 'advanced'
    });

    recommendations.push({
      title: 'Speech Recognition with Whisper',
      path: '/docs/module-4-vla/whisper',
      excerpt: 'Implement voice command recognition using OpenAI Whisper.',
      reason: 'Enable voice control',
      priority: 3,
      category: 'practical'
    });
  }

  // ===== PYTHON EXPERIENCE RECOMMENDATIONS =====
  if (hasResponse('python_experience', 'Yes, comfortable with Python')) {
    // Prioritize Python-heavy modules (ROS 2, Module 4)
    if (!recommendations.some(r => r.path === '/docs/module-1-ros2/')) {
      recommendations.push({
        title: 'Module 1: ROS 2 Fundamentals',
        path: '/docs/module-1-ros2/',
        excerpt: 'Perfect for Python developers - learn ROS 2 with rclpy library integration.',
        reason: 'Great fit for your Python skills',
        priority: 4,
        category: 'fundamentals'
      });
    }
  }

  // ===== NO PYTHON EXPERIENCE =====
  if (hasResponse('python_experience', 'No, but willing to learn')) {
    // Start with intro and basic ROS concepts
    if (!recommendations.some(r => r.path === '/docs/module-1-ros2/nodes-topics')) {
      recommendations.push({
        title: 'ROS 2 Nodes and Topics',
        path: '/docs/module-1-ros2/nodes-topics',
        excerpt: 'Learn ROS 2 fundamentals alongside Python basics.',
        reason: 'Good starting point for learning both',
        priority: 3,
        category: 'getting-started'
      });
    }
  }

  // ===== ADD REFERENCES FOR ALL USERS =====
  recommendations.push({
    title: 'References and Resources',
    path: '/docs/references',
    excerpt: 'Comprehensive list of additional learning resources, papers, and documentation.',
    reason: 'Helpful reference material',
    priority: 2,
    category: 'fundamentals'
  });

  // Remove duplicates and sort by priority
  const uniqueRecommendations = Array.from(
    new Map(recommendations.map(item => [item.path, item])).values()
  );

  return uniqueRecommendations
    .sort((a, b) => b.priority - a.priority)
    .slice(0, 10);
}
