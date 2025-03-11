from django.db.models.signals import post_save
from django.dispatch import receiver
from django.contrib.auth.models import User
from .models import Map

@receiver(post_save, sender=User)
def create_default_map(sender, instance, created, **kwargs):
    if created and not Map.objects.filter(user=instance, is_default=True).exists():
        university_matrix = [
            [0, 2, 2, 0, 1, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 1, 0, 0, 1, 2],
            [0, 0, 0, 0, 2, 0],
            [2, 1, 0, 1, 1, 2]
        ]
        # Create the default map and set it as the current map
        Map.objects.create(
            user=instance,
            name="University Matrix",
            matrix=university_matrix,
            is_default=True,
            is_current=True
        )
