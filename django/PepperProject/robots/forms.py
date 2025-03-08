from django import forms
from .models import Robot

class RobotForm(forms.ModelForm):
    # Define choices for the language field
    LANGUAGE_CHOICES = [
        ('en', 'English'),
        ('fr', 'French'),
    ]

    # Override the language field to use a ChoiceField
    language = forms.ChoiceField(choices=LANGUAGE_CHOICES, widget=forms.Select(attrs={'class': 'form-control'}))

    class Meta:
        model = Robot
        fields = ['name', 'nao_ip', 'network_interface', 'language']

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Add CSS classes to form fields
        self.fields['name'].widget.attrs.update({'class': 'form-control'})
        self.fields['nao_ip'].widget.attrs.update({'class': 'form-control'})
        self.fields['network_interface'].widget.attrs.update({'class': 'form-control'})