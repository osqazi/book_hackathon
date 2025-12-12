// plugins/docusaurus-auth/src/AuthProviderWrapper.js
// Wrapper to provide auth context to the Docusaurus app

import React from 'react';
import AuthProvider from '@site/src/auth/context/AuthProvider';

export default function AuthProviderWrapper({ children }) {
  return <AuthProvider>{children}</AuthProvider>;
}