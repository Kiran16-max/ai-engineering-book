import React from 'react';
import Chatbot from '@site/src/components/Chatbot';

// This component wraps the entire application
export default function Root({children}) {
  return (
    <>
      {children}
      <Chatbot />
    </>
  );
}