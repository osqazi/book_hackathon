// src/pages/test-auth.tsx
import React from 'react';
import SignupForm from '../auth/components/SignupForm';
import SigninForm from '../auth/components/SigninForm';
import { useAuth } from '../auth/hooks/useAuth';

const TestAuthPage: React.FC = () => {
  const { user, isAuthenticated, isLoading } = useAuth();

  const handleSuccess = () => {
    alert('Operation completed successfully!');
  };

  const handleError = (error: string) => {
    alert(`Error: ${error}`);
  };

  return (
    <div style={{ padding: '20px', maxWidth: '600px', margin: '0 auto' }}>
      <h1>Authentication Test Page</h1>

      <div style={{ marginBottom: '30px' }}>
        <h2>Current Status</h2>
        <p><strong>Loading:</strong> {isLoading ? 'Yes' : 'No'}</p>
        <p><strong>Authenticated:</strong> {isAuthenticated ? 'Yes' : 'No'}</p>
        {user && (
          <div>
            <p><strong>User:</strong> {user.email}</p>
            <p><strong>Background Complete:</strong> {user.backgroundComplete ? 'Yes' : 'No'}</p>
          </div>
        )}
      </div>

      <div style={{ marginBottom: '30px' }}>
        <h2>Sign Up</h2>
        <SignupForm onSuccess={handleSuccess} onError={handleError} />
      </div>

      <div>
        <h2>Sign In</h2>
        <SigninForm onSuccess={handleSuccess} onError={handleError} />
      </div>
    </div>
  );
};

export default TestAuthPage;