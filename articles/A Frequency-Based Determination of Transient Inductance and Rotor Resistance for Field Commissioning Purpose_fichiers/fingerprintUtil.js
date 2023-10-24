var FingerprintUtil = (function(FingerprintUtil) { 
	'use strict';

	var COOKIE_KEY = 'xfp';

	function writeCookie(visitorId) {
		var now = new Date();
		now.setTime(now.getTime() + 1 * 3600 * 1000); // Cookie Expires 24 hrs
		document.cookie= COOKIE_KEY + "=" + visitorId + ";path=/;expires=" + now.toUTCString();
	}

	function isCookieExists() {

		var pattern = RegExp(COOKIE_KEY + "=.[^;]*")
		var matched = document.cookie.match(pattern)
		if (matched) {
			var cookie = matched[0].split('=')
			return cookie[0] === COOKIE_KEY;
		}
		return false;
	}
	
	FingerprintUtil = {
		writeCookie: writeCookie,
		isCookieExists: isCookieExists
	};

	return FingerprintUtil;

})(FingerprintUtil || {});