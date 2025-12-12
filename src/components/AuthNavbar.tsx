// src/components/AuthNavbar.tsx
import React from 'react';
import { useAuth } from '../auth/hooks/useAuth';

const AuthNavbar: React.FC = () => {
  const { user, isAuthenticated, isLoading, signOut } = useAuth();

  if (isLoading) {
    return (
      <div className="auth-navbar">
        <span>Loading...</span>
      </div>
    );
  }

  if (isAuthenticated && user) {
    return (
      <div className="auth-navbar auth-navbar--authenticated">
        <span className="auth-navbar__greeting">Hello, {user.email.split('@')[0]}!</span>
        <button
          className="auth-navbar__signout"
          onClick={async () => {
            await signOut();
            window.location.reload(); // Refresh the page to update navbar state
          }}
        >
          Sign Out
        </button>
      </div>
    );
  }

  return (
    <div className="auth-navbar auth-navbar--unauthenticated">
      <a href="/book_hackathon/signin" className="auth-navbar__signin">
        Sign In
      </a>
    </div>
  );
};

export default AuthNavbar;