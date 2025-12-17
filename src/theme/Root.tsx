import React from 'react';
import { AuthProvider } from '@site/src/auth/context/AuthProvider';
import { QueryClientProvider } from '@tanstack/react-query';
import { queryClient } from '@site/src/lib/queryClient';
import '@site/src/styles/personalization.css';

// Root component wraps the entire Docusaurus app
export default function Root({ children }) {
  return (
    <QueryClientProvider client={queryClient}>
      <AuthProvider>{children}</AuthProvider>
    </QueryClientProvider>
  );
}