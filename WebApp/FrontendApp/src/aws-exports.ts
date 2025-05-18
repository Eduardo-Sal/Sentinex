// aws-exports.js (or similar)
const awsmobile = {
  Auth: {
    region: import.meta.env.VITE_AWS_REGION,
    userPoolId: import.meta.env.VITE_AWS_USER_POOL_ID,
    userPoolWebClientId: import.meta.env.VITE_AWS_USER_POOL_WEB_CLIENT_ID,
    identityPoolId: import.meta.env.VITE_AWS_IDENTITY_POOL_ID,
    mandatorySignIn: true,
    signUpVerificationMethod: 'email',
    authenticationFlowType: 'USER_SRP_AUTH'
  },
  };
  
  export default awsmobile;