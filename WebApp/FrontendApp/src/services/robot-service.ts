import { API_CONFIG } from '../config/api-config';
import { PairingResponse } from '../types/robot'; 
import { AWSService } from './aws-service';


export const RobotService = {
    async pairRobot(user_uuid: string, robot_id: number): Promise<PairingResponse> {
        const url = `${API_CONFIG.BASE_URL}/robots/pair`;
        const payload = { user_uuid, robot_id };
    
        const response = await fetch(url, {
          method: 'POST',
          headers: {
            ...API_CONFIG.HEADERS,
            'Content-Type': 'application/json',
            'Accept': 'application/json'
          },
          body: JSON.stringify(payload),
          mode: 'cors',
          credentials: 'include'
        });
    
        if (!response.ok) {
          const errorData = await response.json();
          throw new Error(errorData.detail || 'Pairing failed');
        }
    
        const data: PairingResponse = await response.json();
        return data;
      },

  async unpairRobot(user_uuid: string): Promise<void> {
    const url = `${API_CONFIG.BASE_URL}/robots/unpair`;
    const payload = { user_uuid };

    const response = await fetch(url, {
      method: 'POST',
      headers: {
        ...API_CONFIG.HEADERS,
        'Content-Type': 'application/json',
        'Accept': 'application/json'
      },
      body: JSON.stringify(payload),
      mode: 'cors',
      credentials: 'include'
    });

    if (!response.ok) {
      const errorData = await response.json();
      throw new Error(errorData.detail || 'Unpairing failed');
    }

    localStorage.removeItem('robotId');
  },

  async fetchRobotId(user_uuid: string): Promise<number | null> {
    const url = `${API_CONFIG.BASE_URL}/users/robot-id?user_uuid=${user_uuid}`;
  
    const response = await fetch(url, {
      method: 'GET',
      headers: {
        ...API_CONFIG.HEADERS,
        'Accept': 'application/json'
      },
      mode: 'cors',
      credentials: 'include'
    });
  
    if (!response.ok) {
      return null;
    }
  
    const data = await response.json();
    const robotId = data?.robot_id;
  
    if (robotId) {
      localStorage.setItem('robotId', robotId.toString());
  
      try {
        const awsMetadata = await AWSService.getAwsInfo(user_uuid);
        localStorage.setItem('awsMetadata', JSON.stringify(awsMetadata));
      } catch (err) {
        console.error('Failed to fetch AWS metadata:', err);
      }
    }
  
    return robotId || null;
  }
}
