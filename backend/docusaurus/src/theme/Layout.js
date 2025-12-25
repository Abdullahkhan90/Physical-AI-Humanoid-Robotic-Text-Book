import React from 'react';
import {useColorMode} from '@docusaurus/theme-common';
import Chatbot from '@site/src/components/Chatbot';
import Layout from '@theme-original/Layout';

export default function LayoutWrapper(props) {
  return (
    <Layout {...props}>
      {/* Add the Chatbot component globally */}
      <Chatbot />
      {props.children}
    </Layout>
  );
}