/**
 * Hide the addressbar on ios & android devices
 * https://gist.github.com/yckart/5609969
 *
 * Based on the work from Nate Smith
 * @see https://gist.github.com/nateps/1172490
 *
 * Copyright (c) 2013 Yannick Albert (http://yckart.com)
 * Licensed under the MIT license (http://www.opensource.org/licenses/mit-license.php).
 * 2013/07/10
 */
 
;(function(window) {
 
    window.hideAddressbar = function (elem) {
    
        elem = typeof elem === 'string' ? document.querySelector(elem) : elem;
    
        var ua = navigator.userAgent,
            iphone = ~ua.indexOf('iPhone') || ~ua.indexOf('iPod'),
            ipad = ~ua.indexOf('iPad'),
            ios = iphone || ipad,
            android = ~ua.indexOf('Android'),
            // Detect if this is running as a fullscreen app from the homescreen
            fullscreen = window.navigator.standalone,
            lastWidth = 0;
    
        // We don't go further if we are
        // not on an ios or android device
        // or the passed element was not found
        if (!(ios || android) || !elem) return;
    
        if (android) {
    
            // Android's browser adds the scroll position to the innerHeight, just to
            // make this really fucking difficult. Thus, once we are scrolled, the
            // page height value needs to be corrected in case the page is loaded
            // when already scrolled down. The pageYOffset is of no use, since it always
            // returns 0 while the address bar is displayed.
            window.addEventListener('scroll', function () {
                elem.style.height = window.innerHeight + 'px';
            }, false);
    
        }
    
        var setupScroll = function () {
    
            var height = 0;
    
            // Start out by adding the height of the location bar to the width, so that
            // we can scroll past it
            if (ios) {
    
                // iOS reliably returns the innerWindow size for documentElement.clientHeight
                // but window.innerHeight is sometimes the wrong value after rotating
                // the orientation
                height = document.documentElement.clientHeight;
    
                // Only add extra padding to the height on iphone / ipod, since the ipad
                // browser doesn't scroll off the location bar.
                if (iphone && !fullscreen) height += 60;
    
            } else if (android) {
    
                // The stock Android browser has a location bar height of 56 pixels, but
                // this very likely could be broken in other Android browsers.
                height = window.innerHeight + 56;
    
            }
    
            elem.style.height = height + 'px';
    
            // Scroll after a timeout, since iOS will scroll to the top of the page
            // after it fires the onload event
            setTimeout(scrollTo, 0, 0, 1);
    
        };
    
        (function resize() {
            var pageWidth = elem.offsetWidth;
    
            // Android doesn't support orientation change, so check for when the width
            // changes to figure out when the orientation changes
            if (lastWidth === pageWidth) return;
            lastWidth = pageWidth;
    
            setupScroll();
    
            window.addEventListener('resize', resize, false);
        }());
    
    };
 
}(this));