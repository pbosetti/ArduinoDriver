/*
 * Blink pattern for the Portenta H7.
 */

class BlinkPattern {
public:
  BlinkPattern(unsigned long cycleTime, String pattern)
    : _cycleTime(cycleTime), _pattern(pattern) {
      _patternLength = _pattern.length();
    }

  // Return true if the LED should be on at the given time, false otherwise.
  // argument time is the current time in milliseconds, as returned by millis().
  bool state(unsigned long time) {
    if (_patternLength == 0 || _cycleTime == 0) return false;
    unsigned long stepTime = _cycleTime / _patternLength;
    int step = (time % _cycleTime) / stepTime;
    if (step >= _patternLength) step = _patternLength - 1;
    return _pattern[step] == '#';
  }

private:
  unsigned long _cycleTime = 1000;
  String _pattern = "#-";
  int _patternLength;
};