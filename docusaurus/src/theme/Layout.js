import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import Chatbot from '@site/src/components/Chatbot';

export default function Layout(props) {
  return (
    <OriginalLayout {...props}>
      {props.children}
      <div style={{
        position: 'fixed',
        bottom: '20px',
        right: '20px',
        zIndex: 1000
      }}>
        <Chatbot />
      </div>
    </OriginalLayout>
  );
}