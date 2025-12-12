// src/components/UserProfileNavbar.tsx
import React from 'react';
import { useAuth } from '../auth/hooks/useAuth';

const UserProfileNavbar: React.FC = () => {
  const { user, isAuthenticated, isLoading, signOut } = useAuth();

  if (isLoading) {
    return <div className="navbar-item">Loading...</div>;
  }

  if (isAuthenticated && user) {
    return (
      <div className="navbar-user-profile">
        <span className="navbar-user-greeting">Hello, {user.email.split('@')[0]}!</span>
        <button
          className="navbar-signout-btn"
          onClick={async () => {
            await signOut();
            window.location.href = '/book_hackathon/';
          }}
        >
          Sign Out
        </button>
      </div>
    );
  }

  // Show sign in link if not authenticated
  return (
    <a href="/book_hackathon/signin" className="navbar-signin-link">
      Sign In
    </a>
  );
};

export default UserProfileNavbar;