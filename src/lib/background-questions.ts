// src/lib/background-questions.ts
// Background questions reference data for user profiling

export interface BackgroundQuestion {
  id: string;
  category: 'software' | 'hardware';
  questionText: string;
  options: string[];
  required: boolean;
}

export const backgroundQuestions: BackgroundQuestion[] = [
  {
    id: 'experience_level',
    category: 'software',
    questionText: 'What is your overall robotics experience level?',
    options: ['Complete Beginner', 'Some Basics', 'Intermediate', 'Advanced'],
    required: false,
  },
  {
    id: 'programming_languages',
    category: 'software',
    questionText: 'Which programming languages do you know?',
    options: ['Python', 'C++', 'Java', 'MATLAB', 'None yet'],
    required: false,
  },
  {
    id: 'robotics_topics',
    category: 'software',
    questionText: 'Which robotics topics interest you most? (Select all that apply)',
    options: [
      'Kinematics & Motion Planning',
      'Computer Vision',
      'Machine Learning & AI',
      'Control Systems',
      'ROS/ROS2',
      'Sensor Integration',
      'Hardware Design'
    ],
    required: false,
  },
  {
    id: 'humanoid_interest',
    category: 'hardware',
    questionText: 'What aspects of humanoid robotics interest you?',
    options: [
      'Bipedal Walking',
      'Manipulation & Grasping',
      'Human-Robot Interaction',
      'Balance & Stability',
      'Full Body Control',
      'All of the above'
    ],
    required: false,
  },
  {
    id: 'learning_goals',
    category: 'software',
    questionText: 'What are your learning goals?',
    options: [
      'Build my own humanoid robot',
      'Understand the theory',
      'Work in robotics industry',
      'Academic research',
      'Hobby projects'
    ],
    required: false,
  },
  {
    id: 'prior_platforms',
    category: 'hardware',
    questionText: 'Have you worked with any robotics platforms before?',
    options: [
      'ROS/ROS2',
      'Arduino/Raspberry Pi',
      'Simulation (Gazebo, MuJoCo, etc.)',
      'Commercial robots',
      'No experience yet'
    ],
    required: false,
  },
];

export default backgroundQuestions;