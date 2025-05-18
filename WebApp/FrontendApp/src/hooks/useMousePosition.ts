import { useState, useEffect } from 'react';

interface MousePosition {
  docX: number | null;
  docY: number | null;
}

export const useMousePosition = (): MousePosition => {
  const [position, setPosition] = useState<MousePosition>({ docX: null, docY: null });

  useEffect(() => {
    const updatePosition = (e: MouseEvent) => {
      setPosition({
        docX: e.clientX,
        docY: e.clientY
      });
    };

    window.addEventListener('mousemove', updatePosition);

    return () => {
      window.removeEventListener('mousemove', updatePosition);
    };
  }, []);

  return position;
};