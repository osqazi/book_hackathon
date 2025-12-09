import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget';

// This wraps the entire app layout
const LayoutWrapper = (props) => {
  const { children } = props;

  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
};

export default LayoutWrapper;