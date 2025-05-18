import { API_CONFIG } from '../config/api-config';
import { AnalyticsData } from '../types/analytics';

export class AnalyticsService {
  static async getAnalytics(robotId: number): Promise<AnalyticsData> {
    const url = `${API_CONFIG.BASE_URL}${API_CONFIG.ENDPOINTS.ANALYTICS}/${robotId}/analytics`;

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
      throw new Error(`Failed to fetch analytics data, status: ${response.status}`);
    }

    const data = await response.json();
    localStorage.setItem('analyticsData', JSON.stringify(data));
    return data as AnalyticsData;
  }
}