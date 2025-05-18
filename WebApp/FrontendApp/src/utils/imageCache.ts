interface ImageCache {
  url: string;
  timestamp: number;
  expiryTime: number;
}

const CACHE_DURATION = 24 * 60 * 60 * 1000; // 24 hours in milliseconds

export const ImageCacheService = {
  setImageCache(key: string, url: string): void {
    const cache: ImageCache = {
      url,
      timestamp: Date.now(),
      expiryTime: Date.now() + CACHE_DURATION
    };
    localStorage.setItem(`image_cache_${key}`, JSON.stringify(cache));
  },

  getImageCache(key: string): string | null {
    const cached = localStorage.getItem(`image_cache_${key}`);
    if (!cached) return null;

    const cache: ImageCache = JSON.parse(cached);
    if (Date.now() > cache.expiryTime) {
      localStorage.removeItem(`image_cache_${key}`);
      return null;
    }

    return cache.url;
  },

  clearExpiredCache(): void {
    const keys = Object.keys(localStorage);
    keys.forEach(key => {
      if (key.startsWith('image_cache_')) {
        const cached = localStorage.getItem(key);
        if (cached) {
          const cache: ImageCache = JSON.parse(cached);
          if (Date.now() > cache.expiryTime) {
            localStorage.removeItem(key);
          }
        }
      }
    });
  }
};

// Clear expired cache entries on module load
ImageCacheService.clearExpiredCache();