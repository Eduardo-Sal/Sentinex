import { AwsMetadata } from '../types/aws';
import { API_CONFIG } from '../config/api-config';

export class AWSService {
  static async getAwsInfo(userId: string): Promise<AwsMetadata> {
    const url = `${API_CONFIG.BASE_URL}${API_CONFIG.ENDPOINTS.AWS_INFO}/${userId}`;

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
      throw new Error(`Failed to fetch AWS info, status: ${response.status}`);
    }

    const data = await response.json();
    return data as AwsMetadata;
  }
}