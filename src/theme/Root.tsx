import React from 'react';
import { AuthProvider } from '@site/src/auth/context/AuthProvider';

// Root component wraps the entire Docusaurus app
export default function Root({ children }) {
  return <AuthProvider>{children}</AuthProvider>;
}
