class PromptBuilder:
    def build(self, context):
        return f"""
Instruction: {context['task']}
Robot pose: {context['pose']}
Visible objects: {context['objects']}
Decide the next action.
Return format:
{{type, target?, duration?, confidence}}
"""
