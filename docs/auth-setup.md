# Authentication Setup

This document describes how to set up and use the authentication system in the Humanoid Robotics Book project.

## Overview

The authentication system is built using [Better-Auth](https://better-auth.com/), a comprehensive authentication framework for TypeScript. It provides email/password authentication with additional background profiling to personalize the content based on user expertise.

## Features

- **Email/Password Authentication**: Secure sign up and sign in with email and password
- **Background Profiling**: Collect information about user's software and hardware experience
- **Content Personalization**: Adapt content based on user's background
- **Persistent Sessions**: 7-day session persistence by default
- **Secure Storage**: User data is stored securely with encryption

## Architecture

The authentication system consists of:

1. **Auth Server**: A separate Node.js server running Better-Auth (on port 3001)
2. **Frontend Components**: React components for sign up, sign in, and profile management
3. **Docusaurus Integration**: Plugin that wraps the app with authentication context
4. **Personalization Engine**: Components that adapt content based on user background

## Setup

### Prerequisites

- Node.js 18+ installed
- Docusaurus project set up
- SQLite (development) or PostgreSQL (production) database

### Installation

The authentication dependencies are already installed as part of the project:

```bash
npm install better-auth drizzle-orm drizzle-kit better-sqlite3
```

### Configuration

1. **Environment Variables**:
   Create or update your `.env` file with authentication settings:

   ```
   AUTH_URL=http://localhost:3001
   ```

2. **Run the Auth Server**:
   ```bash
   cd auth-server
   npm install
   npm run dev
   ```

3. **Start the Docusaurus Server**:
   ```bash
   npm start
   ```

## Usage

### Sign Up

1. Navigate to the sign-up page
2. Enter your email and password (minimum 8 characters)
3. Answer background questions about your software and hardware experience
4. Your profile will be used to personalize content

### Sign In

1. Navigate to the sign-in page
2. Enter your email and password
3. You'll be authenticated and content will be personalized based on your profile

### Personalized Content

The `ContentAdapter` component will automatically adapt content based on your background information. Example usage:

```tsx
import ContentAdapter from '@site/src/components/personalization/ContentAdapter';

function MyPage() {
  return (
    <ContentAdapter>
      <div>
        {/* Your content that will be personalized */}
      </div>
    </ContentAdapter>
  );
}
```

## Components

### SignupForm

The `SignupForm` component provides the registration interface with background profiling questions.

```tsx
import SignupForm from '@site/src/auth/components/SignupForm';

function RegisterPage() {
  return <SignupForm onSuccess={() => console.log('Registration successful')} />;
}
```

### SigninForm

The `SigninForm` component provides the login interface.

```tsx
import SigninForm from '@site/src/auth/components/SigninForm';

function LoginPage() {
  return <SigninForm onSuccess={() => console.log('Login successful')} />;
}
```

### ContentAdapter

The `ContentAdapter` component wraps content and adapts it based on user background.

## API Endpoints

The auth server provides the following endpoints (relative to http://localhost:3001):

- `POST /api/auth/sign-up` - Register a new user
- `POST /api/auth/sign-in` - Authenticate a user
- `POST /api/auth/sign-out` - Log out a user
- `GET /api/auth/get-session` - Get current session

## Background Questions

The system collects information about your experience through multiple-choice questions:

### Software Background
- Programming languages familiarity
- Frameworks/libraries experience
- Software development experience level

### Hardware Background
- Robotics platforms experience
- Sensors used in projects
- Actuators worked with
- Hardware development experience level

## Security

- Passwords are hashed using industry-standard algorithms
- Sessions are secure and have a 7-day expiration
- HTTPS is required in production
- Personalization data is stored separately and securely

## Development

### Adding New Background Questions

1. Update the `backgroundQuestions` array in `src/lib/background-questions.ts`
2. The new questions will automatically appear in the registration form

### Adding New Personalized Components

1. Use the `ContentAdapter` component to wrap content
2. Or create new components that use the `useAuth` hook to access user information

### Testing

Run the auth server separately and test authentication flows through the Docusaurus interface.

## Troubleshooting

### Auth Server Not Responding

- Make sure the auth server is running on port 3001
- Check the console for any error messages
- Verify network connectivity between Docusaurus and auth server

### Content Not Personalizing

- Ensure the user is properly authenticated
- Verify that background information has been provided
- Check browser console for any errors

## Environment Configuration

In production, make sure to:

- Use HTTPS for all authentication endpoints
- Configure proper session storage (Redis, etc.)
- Set up proper logging and monitoring
- Implement rate limiting for auth endpoints