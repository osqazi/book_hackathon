// src/pages/signup.tsx
import React, { useState } from 'react';
import Layout from '@theme/Layout';
import SignupForm from '../auth/components/SignupForm';
import '../css/auth-pages.css';

const SignupPage: React.FC = () => {
  const [showSuccessMessage, setShowSuccessMessage] = useState(false);

  return (
    <Layout title="Sign Up" description="Join the Humanoid Robotics community">
      <div className="auth-page">
        <div className="auth-container">
          <div className="auth-header">
            <h1>Create Your Account</h1>
            <p>Join the Humanoid Robotics community and get personalized content</p>
          </div>

          {showSuccessMessage ? (
            <div className="success-message-container">
              <div className="success-message">
                <h2>Thank you for signing up!</h2>
                <p>Now redirecting to your Personalization Page...</p>
              </div>
            </div>
          ) : (
            <div className="signup-form-container">
              <SignupForm
                onSuccess={() => {
                  // Show success message first
                  setShowSuccessMessage(true);
                  // Redirect after a short delay
                  setTimeout(() => {
                    window.location.href = '/book_hackathon/personalization';
                  }, 2000);
                }}
                onError={(error) => {
                  console.error('Signup error:', error);
                  // Handle error appropriately - could show in UI or redirect
                  if (typeof error === 'object' && error !== null && 'message' in error) {
                    alert(`Signup error: ${(error as any).message || 'Unknown error'}`);
                  } else {
                    alert(`Signup error: ${error}`);
                  }
                }}
              />
            </div>
          )}
        </div>
      </div>
    </Layout>
  );
};

export default SignupPage;