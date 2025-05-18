import { Storage } from 'aws-amplify';

export const storageConfig = {
  uploadFile: async (file: File, key: string) => {
    try {
      return await Storage.put(key, file, {
        contentType: file.type,
        level: 'protected'
      });
    } catch (error) {
      console.error('Error uploading file:', error);
      throw error;
    }
  },
  
  getFileUrl: async (key: string) => {
    try {
      return await Storage.get(key, { level: 'protected' });
    } catch (error) {
      console.error('Error getting file URL:', error);
      throw error;
    }
  },
  
  deleteFile: async (key: string) => {
    try {
      return await Storage.remove(key, { level: 'protected' });
    } catch (error) {
      console.error('Error deleting file:', error);
      throw error;
    }
  }
};