export interface AnalyticsData {
    robot_id: number;
    temperature: {
      average: number;
      max: number;
      min: number;
      count: number;
      readings: [string, number][];
    };
    sound: {
      event_count: number;
      events: {
        timestamp: string;
        db_level: number;
      }[];
    };
  }