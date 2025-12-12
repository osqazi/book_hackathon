// src/pages/signin.tsx
import React from 'react';
import Layout from '@theme/Layout';
import SigninForm from '../auth/components/SigninForm';
import '../css/auth-pages.css';

const SigninPage: React.FC = () => {
  return (
    <Layout title="Sign In" description="Sign in to access personalized robotics content">
      <div className="auth-page">
        <div className="auth-container">
          <div className="auth-header">
            <h1>Welcome Back</h1>
            <p>Sign in to access personalized robotics content</p>
          </div>

          <div className="signin-form-container">
            <SigninForm
              onSuccess={() => {
                // Redirect to personalization or dashboard after successful signin
                window.location.href = '/book_hackathon/personalization';
              }}
              onError={(error) => {
                console.error('Signin error:', error);
              }}
            />
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default SigninPage;