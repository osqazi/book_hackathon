// src/models/user.ts
// User data models and interfaces for the Better-Auth implementation

// Define the structure for software background
export interface SoftwareBackground {
  programmingLanguages?: string[];
  frameworks?: string[];
  experienceLevel?: string;
  [key: string]: any; // Allow additional fields as needed
}

// Define the structure for hardware background
export interface HardwareBackground {
  roboticsKits?: string[];
  sensors?: string[];
  actuators?: string[];
  experienceLevel?: string;
  [key: string]: any; // Allow additional fields as needed
}

// Define the user profile interface extending Better-Auth's user
export interface UserProfile {
  id: string;
  email: string;
  createdAt: Date;
  updatedAt: Date;
  lastSignInAt?: Date;
  softwareBackground?: SoftwareBackground;
  hardwareBackground?: HardwareBackground;
  backgroundComplete: boolean;
  // Include any other fields from Better-Auth's user model
  [key: string]: any;
}

// Define the registration data interface
export interface RegistrationData {
  email: string;
  password: string;
  softwareBackground?: SoftwareBackground;
  hardwareBackground?: HardwareBackground;
}

// Define the sign-in data interface
export interface SignInData {
  email: string;
  password: string;
}

// Define the session data interface
export interface SessionData {
  user: UserProfile;
  expiresAt: Date;
  [key: string]: any;
}

// Define the background question interface
export interface BackgroundQuestion {
  id: string;
  category: 'software' | 'hardware';
  questionText: string;
  options: string[];
  required: boolean;
}

// Define the response for background questions
export interface BackgroundQuestionResponse {
  [questionId: string]: string | string[]; // Could be single choice or multiple choice
}