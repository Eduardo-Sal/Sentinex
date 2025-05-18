  // hooks/useRobot.ts
  import { useState, useEffect, useCallback } from 'react';
  import { RobotService } from '../services/robot-service';
  import { PairingResponse } from '../types/robot'; // Make sure this exists
  import { AWSService } from '../services/aws-service';



  export const useRobot = (user_uuid: string) => {
    const [robotId, setRobotId] = useState<string>(() => localStorage.getItem('robotId') || '');
    const [isLoading, setIsLoading] = useState(false);
    const [error, setError] = useState<string | null>(null);

    const fetchRobotId = useCallback(async () => {
      if (!user_uuid || robotId) return;
      setIsLoading(true);
      try {
        const id = await RobotService.fetchRobotId(user_uuid);
        if (id !== null && id !== undefined) {
          localStorage.setItem('robotId', id.toString());
          setRobotId(id.toString());

          // AWS STUFF
          const awsMetadata = await AWSService.getAwsInfo(user_uuid);
          localStorage.setItem('awsMetadata', JSON.stringify(awsMetadata));
          console.log('AWS metadata refreshed after pairing:', awsMetadata);
        }
      } catch {
        setError('Failed to fetch robot ID');
      } finally {
        setIsLoading(false);
      }
    }, [user_uuid, robotId]);

    const pairRobot = useCallback(
      async (robot_id: number): Promise<PairingResponse> => {
        try {
          const response = await RobotService.pairRobot(user_uuid, robot_id);
          if (response?.robot_id != null) {
            localStorage.setItem('robotId', response.robot_id.toString());
            setRobotId(response.robot_id.toString());
    
            // AWS STUFF
            const awsMetadata = await AWSService.getAwsInfo(user_uuid);
            localStorage.setItem('awsMetadata', JSON.stringify(awsMetadata));
            console.log('AWS metadata refreshed after pairing:', awsMetadata);
          }
          return response;
        } catch (e) {
          throw new Error('Pairing failed');
        }
      },
      [user_uuid]
    );
    const unpairRobot = useCallback(async () => {
      try {
        await RobotService.unpairRobot(user_uuid);
        localStorage.removeItem('robotId');
        localStorage.removeItem('awsMetadata');
        setRobotId('');
      } catch {
        throw new Error('Unpairing failed');
      }
    }, [user_uuid]);

    useEffect(() => {
      fetchRobotId();
    }, [fetchRobotId]);

    return {
      robotId,
      setRobotId,
      isLoading,
      error,
      fetchRobotId,
      pairRobot,
      unpairRobot,
    };
  };