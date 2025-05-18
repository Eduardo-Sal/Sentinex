import { useState, useEffect } from "react";

export const useImageUrl = (fileUrl?: string) => {
  const [url, setUrl] = useState<string | undefined>(undefined);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<Error | null>(null);

  useEffect(() => {
    if (!fileUrl) {
      setError(new Error("Invalid file URL"));
      setIsLoading(false);
      return;
    }

    console.log("Using existing file_url:", fileUrl);  // ✅ Debugging log

    // ✅ Directly set the file URL instead of making an API call
    setUrl(fileUrl);
    setIsLoading(false);
  }, [fileUrl]);

  return { url, isLoading, error };
};