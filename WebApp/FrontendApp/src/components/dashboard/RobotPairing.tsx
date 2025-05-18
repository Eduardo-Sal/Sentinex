import React, { useState } from 'react';
import { motion } from 'framer-motion';
import { Smartphone, Check, AlertCircle } from 'lucide-react';
import { AWSService } from '../../services/aws-service';

const RobotPairing = () => {
  const [robotId, setRobotId] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState(false);

  const handlePairRobot = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsLoading(true);
    setError(null);

    try {
      await AWSService.pairRobot(robotId);
      setSuccess(true);
      setRobotId('');
    } catch (err) {
      setError('Failed to pair robot. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="p-6">
      <h2 className="text-2xl font-semibold text-gray-900 mb-6">Pair New Robot</h2>

      <div className="max-w-md">
        <form onSubmit={handlePairRobot} className="space-y-6">
          {error && (
            <motion.div
              initial={{ opacity: 0, y: -10 }}
              animate={{ opacity: 1, y: 0 }}
              className="bg-red-50 border border-red-200 rounded-md p-4 flex items-center"
            >
              <AlertCircle className="h-5 w-5 text-red-500 mr-2" />
              <span className="text-red-700 text-sm">{error}</span>
            </motion.div>
          )}

          {success && (
            <motion.div
              initial={{ opacity: 0, y: -10 }}
              animate={{ opacity: 1, y: 0 }}
              className="bg-green-50 border border-green-200 rounded-md p-4 flex items-center"
            >
              <Check className="h-5 w-5 text-green-500 mr-2" />
              <span className="text-green-700 text-sm">Robot paired successfully!</span>
            </motion.div>
          )}

          <div>
            <label htmlFor="robotId" className="block text-sm font-medium text-gray-700">
              Robot ID
            </label>
            <div className="mt-1 relative">
              <div className="absolute inset-y-0 left-0 pl-3 flex items-center pointer-events-none">
                <Smartphone className="h-5 w-5 text-gray-400" />
              </div>
              <input
                type="text"
                id="robotId"
                value={robotId}
                onChange={(e) => setRobotId(e.target.value)}
                className="block w-full pl-10 pr-3 py-2 border border-gray-300 rounded-md shadow-sm focus:ring-black focus:border-black sm:text-sm"
                placeholder="Enter Robot ID"
                required
              />
            </div>
            <p className="mt-2 text-sm text-gray-500">
              Enter the unique identifier found on your robot's label
            </p>
          </div>

          <motion.button
            whileHover={{ scale: 1.02 }}
            whileTap={{ scale: 0.98 }}
            type="submit"
            disabled={isLoading}
            className="w-full flex justify-center py-2 px-4 border border-transparent rounded-md shadow-sm text-sm font-medium text-white bg-black hover:bg-gray-800 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-black disabled:opacity-50 disabled:cursor-not-allowed"
          >
            {isLoading ? 'Pairing...' : 'Pair Robot'}
          </motion.button>
        </form>
      </div>
    </div>
  );
};

export default RobotPairing;