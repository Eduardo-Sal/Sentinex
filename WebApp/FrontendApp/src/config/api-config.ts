export const API_CONFIG = {
    // Use the WebContainer URL format for the API
  BASE_URL: import.meta.env.VITE_API_BASE_URL || 'https://oci3-szdoq06p--8000--eb2a6bdc.local-credentialless.webcontainer.io/api',
  ENDPOINTS: {
    NOTIFICATIONS: '/notifications/user',
    PRESIGNED_URL: '/presigned-url',
    AWS_INFO: '/aws/awsInfo',
    ANALYTICS: '/sensor',

    // 🔧 Robot-related endpoints
    ROBOT: {
      PAIR: '/robots/pair',
      UNPAIR: '/robots/unpair',
      FETCH: '/users/robot', // POST with { user_uuid }
    }
  },
  HEADERS: {
    'Content-Type': 'application/json',
    'Accept': 'application/json'
  }
};

export const getApiUrl = (endpoint: string): string => {
  return `${API_CONFIG.BASE_URL}${endpoint}`;
};

export const getPresignedUrl = async (imageKey: string): Promise<string> => {
  try {
    const response = await fetch(getApiUrl(API_CONFIG.ENDPOINTS.PRESIGNED_URL), {
      method: 'POST',
      headers: API_CONFIG.HEADERS,
      body: JSON.stringify({ key: imageKey })
    });
    
    if (!response.ok) {
      throw new Error('Failed to get presigned URL');
    }

    const data = await response.json();
    return data.url;
  } catch (error) {
    console.error('Error getting presigned URL:', error);
    throw error;
  }
};