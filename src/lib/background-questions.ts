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
    id: 'q1',
    category: 'software',
    questionText: 'Which programming languages are you familiar with?',
    options: ['Python', 'C++', 'JavaScript', 'C#', 'Java', 'ROS', 'Other'],
    required: false,
  },
  {
    id: 'q2',
    category: 'software',
    questionText: 'Which frameworks/libraries have you used?',
    options: ['React', 'Vue', 'Angular', 'ROS 2', 'OpenCV', 'PyTorch', 'TensorFlow', 'Other'],
    required: false,
  },
  {
    id: 'q3',
    category: 'software',
    questionText: 'What is your experience level with software development?',
    options: ['Beginner', 'Intermediate', 'Advanced', 'Expert'],
    required: false,
  },
  {
    id: 'q4',
    category: 'hardware',
    questionText: 'Which robotics platforms have you worked with?',
    options: ['ROS', 'ROS 2', 'Arduino', 'Raspberry Pi', 'NVIDIA Jetson', 'Other'],
    required: false,
  },
  {
    id: 'q5',
    category: 'hardware',
    questionText: 'Which sensors have you used in robotics projects?',
    options: ['Cameras', 'LIDAR', 'IMU', 'Ultrasonic', 'GPS', 'Force/Torque', 'Other'],
    required: false,
  },
  {
    id: 'q6',
    category: 'hardware',
    questionText: 'Which actuators have you worked with?',
    options: ['Servo Motors', 'Stepper Motors', 'DC Motors', 'Pneumatics', 'Hydraulics', 'Other'],
    required: false,
  },
  {
    id: 'q7',
    category: 'hardware',
    questionText: 'What is your experience level with hardware development?',
    options: ['Beginner', 'Intermediate', 'Advanced', 'Expert'],
    required: false,
  },
];

export default backgroundQuestions;