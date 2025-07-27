#ifndef __SHOW_HPP
#define __SHOW_HPP

class Show {
public:
    // Display information on OLED
    static void oledShow();
    
    // Application display function
    static void appShow();
    
    // Data scope display
    static void dataScope();
    
    // Utility function - absolute value
    static int myabs(int a);
};

#endif // __SHOW_HPP