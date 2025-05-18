import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';
import { Amplify } from 'aws-amplify';
import App from './App.tsx';
import './index.css';
//import { configureAmplify } from './config/aws-config';
import { AWSIoTProvider } from '@aws-amplify/pubsub';


// Initialize Amplify configuration
//configureAmplify();


createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <App />
  </StrictMode>
);