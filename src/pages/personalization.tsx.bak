// src/pages/personalization.tsx
import React from 'react';
import Layout from '@theme/Layout';
import ContentAdapter from '../components/personalization/ContentAdapter';
import { useAuth } from '../auth/hooks/useAuth';
import '../css/auth-pages.css';

const PersonalizationPage: React.FC = () => {
  const { user, isAuthenticated } = useAuth();

  return (
    <Layout title="Personalized Content" description="Get personalized robotics content based on your background">
      <div className="personalization-page">
        <div className="personalization-container">
          <div className="personalization-header">
            <h1>Personalized Content</h1>
            {isAuthenticated && user && (
              <div className="user-info">
                <p>Welcome, <strong>{user.email}</strong>!</p>
              </div>
            )}
          </div>

          <ContentAdapter
            fallback={
              <div className="personalization-fallback">
                <h2>Sign in to see personalized content</h2>
                <p>Please <a href="/book_hackathon/signin">sign in</a> or <a href="/book_hackathon/signup">sign up</a> to get personalized content recommendations based on your background in robotics, programming, and hardware development.</p>
              </div>
            }
          >
            <div className="personalized-content">
              <h2>Recommended Content for You</h2>
              <div className="content-grid">
                <div className="content-card">
                  <h3>Beginner: ROS 2 Basics</h3>
                  <p>Introduction to ROS 2 concepts, nodes, topics, and services for newcomers to robotics development.</p>
                </div>
                <div className="content-card">
                  <h3>Intermediate: Navigation Systems</h3>
                  <p>Learn how to implement navigation systems for humanoid robots with path planning and obstacle avoidance.</p>
                </div>
                <div className="content-card">
                  <h3>Advanced: AI Integration</h3>
                  <p>Deep dive into integrating AI models with humanoid robot control systems for perception and decision making.</p>
                </div>
              </div>
            </div>
          </ContentAdapter>
        </div>
      </div>
    </Layout>
  );
};

export default PersonalizationPage;