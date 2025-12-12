// src/pages/test-personalization.tsx
import React from 'react';
import ContentAdapter from '../components/personalization/ContentAdapter';
import { useAuth } from '../auth/hooks/useAuth';

const TestPersonalizationPage: React.FC = () => {
  const { user, isAuthenticated, isLoading } = useAuth();

  return (
    <div style={{ padding: '20px', maxWidth: '800px', margin: '0 auto' }}>
      <h1>Personalization Test Page</h1>

      <div style={{ marginBottom: '30px' }}>
        <h2>User Status</h2>
        <p><strong>Loading:</strong> {isLoading ? 'Yes' : 'No'}</p>
        <p><strong>Authenticated:</strong> {isAuthenticated ? 'Yes' : 'No'}</p>
        {user && (
          <div>
            <p><strong>User:</strong> {user.email}</p>
            <p><strong>Software Background:</strong> {JSON.stringify(user.softwareBackground || 'None')}</p>
            <p><strong>Hardware Background:</strong> {JSON.stringify(user.hardwareBackground || 'None')}</p>
            <p><strong>Background Complete:</strong> {user.backgroundComplete ? 'Yes' : 'No'}</p>
          </div>
        )}
      </div>

      <ContentAdapter
        fallback={
          <div style={{ border: '1px solid #ccc', padding: '15px', borderRadius: '5px' }}>
            <h3>Generic Content</h3>
            <p>This is the generic content shown to unauthenticated users or users without background data.</p>
          </div>
        }
      >
        <div style={{ border: '1px solid #007cba', padding: '15px', borderRadius: '5px' }}>
          <h3>Personalized Content</h3>
          <p>This content is personalized based on your background information.</p>
          <div style={{ marginTop: '15px' }}>
            <h4>Advanced Robotics Tutorial</h4>
            <p>Since you have experience with robotics, here's an advanced tutorial on humanoid locomotion algorithms...</p>
          </div>
        </div>
      </ContentAdapter>
    </div>
  );
};

export default TestPersonalizationPage;