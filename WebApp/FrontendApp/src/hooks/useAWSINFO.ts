import { AwsMetadata } from '../types/aws';
import { useState, useEffect, useCallback } from 'react';
import { AWSService } from '../services/aws-service';

export function useAwsInfo(userId: string) {
  const [awsInfo, setAwsInfo] = useState<AwsMetadata | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  const fetchAwsInfo = useCallback(async () => {
    if (!userId) return;

    setLoading(true);
    try {
      const data = await AWSService.getAwsInfo(userId);
      setAwsInfo(data);
      setError(null);
    } catch (err: any) {
      console.error('Failed to fetch AWS info:', err);
      setError(err.message);
    } finally {
      setLoading(false);
    }
  }, [userId]);

  useEffect(() => {
    fetchAwsInfo();
  }, [fetchAwsInfo]);

  return { awsInfo, loading, error, refetchAwsInfo: fetchAwsInfo };
}