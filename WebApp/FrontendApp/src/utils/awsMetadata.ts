export const getCachedAwsMetadata = () => {
    const raw = localStorage.getItem('awsMetadata');
    if (!raw) return null;
  
    try {
      return JSON.parse(raw);
    } catch {
      console.warn('Corrupted AWS metadata in cache');
      return null;
    }
  };