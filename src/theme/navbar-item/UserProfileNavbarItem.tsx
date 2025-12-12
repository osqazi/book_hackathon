// src/theme/navbar-item/UserProfileNavbarItem.tsx
import React, { useEffect, useState } from 'react';
import { useAuth } from '../../auth/hooks/useAuth';

const UserProfileNavbarItem = () => {
  const { user, isAuthenticated, isLoading, signOut } = useAuth();
  const [mounted, setMounted] = useState(false);

  useEffect(() => {
    setMounted(true);
  }, []);

  if (!mounted || isLoading) {
    return <div className="navbar__item">Loading...</div>;
  }

  if (isAuthenticated && user) {
    return (
      <div className="navbar__item navbar__user-profile">
        <span className="navbar__user-greeting">Hello, {user.email.split('@')[0]}!</span>
        <button
          className="navbar__signout-btn"
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

  return (
    <a href="/book_hackathon/signin" className="navbar__item navbar__signin-link">
      Sign In
    </a>
  );
};

export default UserProfileNavbarItem;