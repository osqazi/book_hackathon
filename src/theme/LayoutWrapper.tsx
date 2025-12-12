import React, { useEffect } from 'react';
import ChatWidget from '@site/src/components/ChatWidget';
import { useAuth } from '@site/src/auth/hooks/useAuth';

// This wraps the entire app layout
const LayoutWrapper = (props) => {
  const { children } = props;
  const { user, isAuthenticated, isLoading, signOut } = useAuth();

  console.log('LayoutWrapper render - isLoading:', isLoading, 'isAuthenticated:', isAuthenticated, 'user:', user);

  useEffect(() => {
    console.log('LayoutWrapper useEffect - isLoading:', isLoading, 'isAuthenticated:', isAuthenticated, 'user:', user);

    if (isLoading) return;

    // Add a small delay to ensure DOM is ready
    const timeoutId = setTimeout(() => {
      // Find all navbar items
      const navbarItems = document.querySelectorAll('.navbar__item');
      console.log('Found navbar items:', navbarItems.length);

      navbarItems.forEach((item) => {
        const link = item.querySelector('a');
        if (!link) return;

        const href = link.getAttribute('href');

        if (isAuthenticated) {
          // Hide Sign Up and Sign In links when authenticated
          if (href?.includes('/signup') || href?.includes('/signin')) {
            console.log('Hiding link:', href);
            (item as HTMLElement).style.display = 'none';
          }
        } else {
          // Show Sign Up and Sign In links when not authenticated
          if (href?.includes('/signup') || href?.includes('/signin')) {
            console.log('Showing link:', href);
            (item as HTMLElement).style.display = '';
          }
        }
      });

      // Add or remove Sign Out button
      const navbar = document.querySelector('.navbar__items--right');
      console.log('Found navbar container:', !!navbar);
      let signOutBtn = document.getElementById('navbar-signout-btn');

      if (isAuthenticated && user) {
        console.log('User is authenticated, should show Sign Out button');
        if (!signOutBtn) {
          console.log('Creating Sign Out button');
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
            console.log('Sign Out button inserted before GitHub link');
          } else if (navbar) {
            navbar.appendChild(signOutContainer);
            console.log('Sign Out button appended to navbar');
          }
        } else {
          console.log('Sign Out button already exists');
        }
      } else {
        console.log('User not authenticated, removing Sign Out button if exists');
        // Remove Sign Out button if user is not authenticated
        if (signOutBtn) {
          signOutBtn.remove();
          console.log('Sign Out button removed');
        }
      }
    }, 100); // Small delay to ensure DOM is ready

    return () => clearTimeout(timeoutId);
  }, [isAuthenticated, isLoading, user, signOut]);

  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
};

export default LayoutWrapper;