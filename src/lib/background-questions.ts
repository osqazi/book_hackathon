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
    questionText: 'What is your robotics experience level?',
    options: ['Complete Beginner', 'Some Experience', 'Advanced'],
    required: false,
  },
  {
    id: 'python_experience',
    category: 'software',
    questionText: 'Do you know Python programming?',
    options: ['Yes, comfortable with Python', 'Some basics', 'No, but willing to learn'],
    required: false,
  },
  {
    id: 'primary_interest',
    category: 'software',
    questionText: 'Which module interests you most?',
    options: [
      'ROS 2 Fundamentals',
      'Simulation & Physics',
      'AI Navigation (NVIDIA Isaac)',
      'Vision-Language-Action (LLM/AI)',
      'All modules equally'
    ],
    required: false,
  },
];

export default backgroundQuestions;