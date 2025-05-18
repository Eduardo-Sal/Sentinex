import { Amplify } from 'aws-amplify';
//import { AWSIoTProvider } from '@aws-amplify/pubsub';
import { AWSIoTProvider } from '@aws-amplify/pubsub';


export const configureAmplify = () => {
  const config = {
    Auth: {
      region: import.meta.env.VITE_AWS_REGION,
      userPoolId: import.meta.env.VITE_AWS_USER_POOL_ID,
      userPoolWebClientId: import.meta.env.VITE_AWS_USER_POOL_WEB_CLIENT_ID,
      identityPoolId: import.meta.env.VITE_AWS_IDENTITY_POOL_ID,
      mandatorySignIn: true,
      signUpVerificationMethod: 'email',
      authenticationFlowType: 'USER_SRP_AUTH'
    },
    PubSub: {
      aws_pubsub_region: import.meta.env.VITE_AWS_REGION,
      aws_pubsub_endpoint: import.meta.env.VITE_AWS_IOT_ENDPOINT
    }
  };

  if (import.meta.env.VITE_AWS_API_ENDPOINT) {
    config['API'] = {
      endpoints: [
        {
          name: 'sentinexApi',
          endpoint: import.meta.env.VITE_AWS_API_ENDPOINT,
          region: import.meta.env.VITE_AWS_REGION
        }
      ]
    };
  }

  

  // Initialize Amplify with config
  Amplify.configure(config);
  // Plug AWS IoT Provider into Amplify

  Amplify.addPluggable(
    new AWSIoTProvider({
      aws_pubsub_region: 'us-east-2',
      aws_pubsub_endpoint:
        'wss://a3kh46o9tgih2p-ats.iot.us-east-2.amazonaws.com/mqtt'
        
    })
  );
};