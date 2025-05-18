import React, { useState, useEffect, useRef } from 'react';
import { Star, ChevronLeft, ChevronRight } from 'lucide-react';

const Testimonials = () => {
  const [currentIndex, setCurrentIndex] = useState(0);
  const sectionRef = useRef<HTMLDivElement>(null);
  const [isVisible, setIsVisible] = useState(false);

  const testimonials = [
    {
      quote: "Sentinex has transformed our campus security. The robots patrol areas that were previously difficult to monitor, and the AI detection has already prevented several break-in attempts.",
      author: "Dr. Sarah Chen",
      position: "Security Director",
      company: "Westlake University",
      image: "https://images.unsplash.com/photo-1573496359142-b8d87734a5a2?ixlib=rb-4.0.3&auto=format&fit=crop&w=200&q=80",
      rating: 5
    },
    {
      quote: "As a mall with over 200 stores, traditional security was always challenging. Sentinex provides consistent coverage throughout our property, even after hours. The mobile app gives us peace of mind 24/7.",
      author: "Michael Rodriguez",
      position: "Operations Manager",
      company: "Meridian Shopping Center",
      image: "https://images.unsplash.com/photo-1560250097-0b93528c311a?ixlib=rb-4.0.3&auto=format&fit=crop&w=200&q=80",
      rating: 5
    },
    {
      quote: "The ROI with Sentinex has been remarkable. We've reduced security personnel costs by 40% while improving coverage and response times. The robots have become a welcomed presence in our facility.",
      author: "Jennifer Park",
      position: "CFO",
      company: "Axiom Pharmaceuticals",
      image: "https://images.unsplash.com/photo-1580489944761-15a19d654956?ixlib=rb-4.0.3&auto=format&fit=crop&w=200&q=80",
      rating: 4
    },
    {
      quote: "Our warehouse complex is over 500,000 square feet. Before Sentinex, we couldn't effectively monitor the entire space. Now, our security team receives real-time alerts and can respond immediately to any situation.",
      author: "Robert Thompson",
      position: "Head of Security",
      company: "Global Distribution Inc.",
      image: "https://images.unsplash.com/photo-1472099645785-5658abf4ff4e?ixlib=rb-4.0.3&auto=format&fit=crop&w=200&q=80",
      rating: 5
    }
  ];

  useEffect(() => {
    const observer = new IntersectionObserver(
      (entries) => {
        if (entries[0].isIntersecting) {
          setIsVisible(true);
        }
      },
      { threshold: 0.1 }
    );

    if (sectionRef.current) {
      observer.observe(sectionRef.current);
    }

    return () => {
      if (sectionRef.current) {
        observer.unobserve(sectionRef.current);
      }
    };
  }, []);

  const nextTestimonial = () => {
    setCurrentIndex((prevIndex) => (prevIndex + 1) % testimonials.length);
  };

  const prevTestimonial = () => {
    setCurrentIndex((prevIndex) => (prevIndex - 1 + testimonials.length) % testimonials.length);
  };

  return (
    <section id="testimonials" className="py-20 bg-black" ref={sectionRef}>
      <div className="container mx-auto px-4 md:px-6">
        <div className="text-center mb-16">
          <h2 className="text-3xl md:text-4xl font-bold mb-4">What Our Clients Say</h2>
          <p className="text-xl text-gray-400 max-w-3xl mx-auto">
            Hear from organizations that have transformed their security with Sentinex.
          </p>
        </div>

        <div className={`transition-all duration-1000 ${isVisible ? 'opacity-100 translate-y-0' : 'opacity-0 translate-y-20'}`}>
          <div className="relative max-w-4xl mx-auto">
            {/* Navigation Buttons */}
            <button 
              className="absolute left-0 top-1/2 transform -translate-y-1/2 -translate-x-12 bg-gray-800 rounded-full p-2 text-white hover:bg-gray-700 transition-colors z-10 hidden md:block"
              onClick={prevTestimonial}
            >
              <ChevronLeft className="h-6 w-6" />
            </button>
            
            <button 
              className="absolute right-0 top-1/2 transform -translate-y-1/2 translate-x-12 bg-gray-800 rounded-full p-2 text-white hover:bg-gray-700 transition-colors z-10 hidden md:block"
              onClick={nextTestimonial}
            >
              <ChevronRight className="h-6 w-6" />
            </button>

            {/* Testimonial Card */}
            <div className="bg-gray-900 rounded-lg p-8 md:p-12 shadow-xl">
              <div className="flex flex-col md:flex-row gap-8">
                <div className="md:w-1/3 flex flex-col items-center">
                  <div className="w-24 h-24 rounded-full overflow-hidden mb-4">
                    <img 
                      src={testimonials[currentIndex].image} 
                      alt={testimonials[currentIndex].author} 
                      className="w-full h-full object-cover"
                    />
                  </div>
                  <h4 className="text-xl font-semibold text-center">{testimonials[currentIndex].author}</h4>
                  <p className="text-gray-400 text-center">{testimonials[currentIndex].position}</p>
                  <p className="text-gray-400 text-center mb-4">{testimonials[currentIndex].company}</p>
                  <div className="flex">
                    {[...Array(5)].map((_, i) => (
                      <Star 
                        key={i} 
                        className={`h-5 w-5 ${i < testimonials[currentIndex].rating ? 'text-yellow-500 fill-yellow-500' : 'text-gray-600'}`} 
                      />
                    ))}
                  </div>
                </div>
                <div className="md:w-2/3">
                  <div className="h-full flex flex-col justify-center">
                    <blockquote className="text-xl italic leading-relaxed mb-6">
                      "{testimonials[currentIndex].quote}"
                    </blockquote>
                  </div>
                </div>
              </div>
            </div>

            {/* Mobile Navigation */}
            <div className="flex justify-center mt-8 md:hidden">
              <button 
                className="bg-gray-800 rounded-full p-2 text-white hover:bg-gray-700 transition-colors mx-2"
                onClick={prevTestimonial}
              >
                <ChevronLeft className="h-6 w-6" />
              </button>
              
              <button 
                className="bg-gray-800 rounded-full p-2 text-white hover:bg-gray-700 transition-colors mx-2"
                onClick={nextTestimonial}
              >
                <ChevronRight className="h-6 w-6" />
              </button>
            </div>

            {/* Indicators */}
            <div className="flex justify-center mt-8">
              {testimonials.map((_, index) => (
                <button
                  key={index}
                  className={`w-3 h-3 mx-1 rounded-full ${
                    currentIndex === index ? 'bg-white' : 'bg-gray-600'
                  }`}
                  onClick={() => setCurrentIndex(index)}
                />
              ))}
            </div>
          </div>
        </div>
      </div>
    </section>
  );
};

export default Testimonials;