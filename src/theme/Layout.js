import React, { useEffect } from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatWidget from '@site/src/components/ChatWidget';
import { useAuth } from '@site/src/auth/hooks/useAuth';

export default function Layout(props) {
  const { user, isAuthenticated, isLoading, signOut } = useAuth();

  useEffect(() => {
    if (isLoading) return;

    // Add a small delay to ensure DOM is ready
    const timeoutId = setTimeout(() => {
      try {
        // Find all navbar items
        const navbarItems = document.querySelectorAll('.navbar__item');

      navbarItems.forEach((item) => {
        // The item itself might be an anchor tag
        const href = item.getAttribute('href') || item.querySelector('a')?.getAttribute('href');

        if (!href) return;

        if (isAuthenticated) {
          // Hide Sign Up and Sign In links when authenticated
          if (href.includes('/signup') || href.includes('/signin')) {
            item.style.display = 'none';
          }
        } else {
          // Show Sign Up and Sign In links when not authenticated
          if (href.includes('/signup') || href.includes('/signin')) {
            item.style.display = '';
          }
        }
      });

      // Add or remove Sign Out button
      const navbar = document.querySelector('.navbar__items--right');
      let signOutBtn = document.getElementById('navbar-signout-btn');

      if (isAuthenticated && user) {
        if (!signOutBtn) {
          // Create Sign Out button if it doesn't exist
          const signOutContainer = document.createElement('div');
          signOutContainer.id = 'navbar-signout-btn';
          signOutContainer.className = 'navbar__item';
          signOutContainer.style.display = 'flex';
          signOutContainer.style.alignItems = 'center';
          signOutContainer.style.gap = '10px';

          const greeting = document.createElement('span');
          greeting.textContent = `Hello, ${user.email.split('@')[0]}!`;
          greeting.style.fontWeight = '500';
          greeting.style.marginRight = '10px';

          const button = document.createElement('button');
          button.textContent = 'Sign Out';
          button.className = 'button button--sm button--danger';
          button.style.background = '#dc3545';
          button.style.color = 'white';
          button.style.border = 'none';
          button.style.padding = '6px 12px';
          button.style.borderRadius = '4px';
          button.style.cursor = 'pointer';
          button.style.fontSize = '0.9rem';
          button.style.fontWeight = '500';

          button.addEventListener('click', async () => {
            await signOut();
            window.location.href = '/book_hackathon/';
          });

          signOutContainer.appendChild(greeting);
          signOutContainer.appendChild(button);

          // Insert before the GitHub link (last item)
          const githubItem = Array.from(navbar?.children || []).find((child) => {
            const link = child.querySelector('a');
            return link?.getAttribute('href')?.includes('github.com');
          });

          if (githubItem && navbar) {
            navbar.insertBefore(signOutContainer, githubItem);
          } else if (navbar) {
            navbar.appendChild(signOutContainer);
          }
        }
      } else {
        // Remove Sign Out button if user is not authenticated
        if (signOutBtn) {
          signOutBtn.remove();
        }
      }
      } catch (error) {
        console.error('Error in navbar manipulation:', error);
      }
    }, 100); // Small delay to ensure DOM is ready

    return () => clearTimeout(timeoutId);
  }, [isAuthenticated, isLoading, user, signOut]);

  return (
    <>
      <OriginalLayout {...props} />
      <ChatWidget />
    </>
  );
}