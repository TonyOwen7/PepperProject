from django.db import models
from django.contrib.auth.models import User

class Robot(models.Model):
    # Define choices for the language field
    LANGUAGE_CHOICES = [
        ('en', 'English'),
        ('fr', 'French'),
    ]

    user = models.ForeignKey(User, on_delete=models.CASCADE, related_name='robots')
    name = models.CharField(max_length=100)
    nao_ip = models.CharField(max_length=15)
    network_interface = models.CharField(max_length=50)
    language = models.CharField(max_length=2, choices=LANGUAGE_CHOICES, default='en')  # Add choices and default

    def __str__(self):
        return self.name