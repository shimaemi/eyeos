import subprocess

def espeak(text):
    """
    Uses espeak with settings for a more natural voice.
    Args:
        text (str): Text to speak.
    """
    # Command with parameters for better sound
    cmd = [
        'espeak',
        '-v', 'en-us',      # American English voice
        '-s', '145',        # Slightly slower speed (145 WPM)
        '-p', '50',         # Medium pitch (50/99)
        '-g', '8',          # Word gap (pause between words, 8ms)
        '-k', '20',         # Emphasis on capital letters (for names/important words)
        text
    ]
    subprocess.Popen(cmd)  # Run asynchronously (won't block code)

# Example usage
espeak("Hello, I am your AI camera. I see a person and a bicycle.")
