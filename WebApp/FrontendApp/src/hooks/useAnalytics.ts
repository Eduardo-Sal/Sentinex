import { useEffect, useState } from 'react';
import { AnalyticsService } from '../services/analytics';
import { AnalyticsData } from '../types/analytics';

export const useAnalytics = (robotId: string) => {
  const [analytics, setAnalytics] = useState<AnalyticsData | null>(() => {
    const saved = localStorage.getItem('analyticsData');
    return saved ? JSON.parse(saved) : null;
  });

  const fetchAnalytics = async () => {
    try {
      const data = await AnalyticsService.getAnalytics(Number(robotId));
      setAnalytics(data);
      localStorage.setItem('analyticsData', JSON.stringify(data));
    } catch (err) {
      console.error('Failed to fetch analytics:', err);
    }
  };

  useEffect(() => {
    if (robotId) {
      fetchAnalytics();
    }
  }, [robotId]);

  return { analytics, refetch: fetchAnalytics };
};